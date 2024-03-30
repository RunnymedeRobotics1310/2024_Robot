package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Shooting;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.SpeakerShooterPolynomialAngleCalc;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

/**
 * Move arm to speaker shoot pose
 * Set shooter speed (distance based)
 */
public class ShootSpeakerFromAnywhereCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_UNLOCK, REVERSE_NOTE, START_SHOOTER, START_FEEDER, FINISHED
    };

    private SwerveSubsystem     swerveSubsystem;
    private LightingSubsystem   lighting;

    private State               state               = State.MOVE_TO_UNLOCK;
    double                      intakeStartPosition = 0;
    private double              lastDistanceToTarget = -1310;
    private boolean             tooClose             = false;
    private Constants.BotTarget botTarget;


    public ShootSpeakerFromAnywhereCommand(ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem,
        LightingSubsystem lighting) {
        super(armSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.lighting        = lighting;
    }

    @Override
    public void initialize() {
        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            log("No note detected in robot. AimSpeakerCommand cancelled");
            state = State.FINISHED;
            return;
        }

        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            botTarget = Constants.BotTarget.BLUE_SPEAKER;
        }
        else {
            botTarget = Constants.BotTarget.RED_SPEAKER;
        }

        lighting.addPattern(SIGNAL, Shooting.getInstance());

        logCommandStart();

        // Use standard Shoot if we're close enough to the speaker
        if (getDistanceToTarget() < 1.6) {
            tooClose = true;
            state = State.REVERSE_NOTE;
        }
        else if (isAtArmPosition(ArmConstants.COMPACT_ARM_POSITION, 2)) {
            state = State.MOVE_TO_UNLOCK;
        }
        else {
            state = State.REVERSE_NOTE;
        }

        intakeStartPosition = armSubsystem.getIntakePosition();
    }

    private double getDistanceToTarget() {
        Pose2d botPose = swerveSubsystem.getPose();
        double distanceToTarget = botPose.getTranslation().getDistance(botTarget.getLocation().toTranslation2d());
        lastDistanceToTarget = distanceToTarget;
        return distanceToTarget;
    }

    private boolean driveArmToCalculatedAngle() {
        // Drive to the arm position at the same time
        double linkAngle = ArmConstants.SHOOT_SPEAKER_PODIUM_ARM_POSITION.linkAngle;
        double distanceToTarget = getDistanceToTarget();
        double aimAngle = SpeakerShooterPolynomialAngleCalc.calculateAimAngle(distanceToTarget);
        Constants.ArmPosition armPositionNew = new Constants.ArmPosition(linkAngle, aimAngle);

        return driveToArmPosition(armPositionNew, 2, 2);
    }

    @Override
    public void execute() {

        final boolean atArmAngle;

        switch (state) {

        case MOVE_TO_UNLOCK:

            // Run the link motor back (up) for .15 seconds to unlock the arm
            armSubsystem.setLinkPivotSpeed(.3);
            armSubsystem.setAimPivotSpeed(0);

            if (isStateTimeoutExceeded(.2)) {
                logStateTransition("Unlock -> Move To Speaker", "Arm Unlocked");
                state = State.REVERSE_NOTE;
            }

            break;

        case REVERSE_NOTE:

            armSubsystem.setShooterSpeed(-0.1);
            armSubsystem.setIntakeSpeed(-0.3);

            // Reverse the note for a number of rotations
            if (Math.abs(armSubsystem.getIntakePosition() - intakeStartPosition) > 2) {
                logStateTransition("Reverse -> Start Shooter", "Shooter Reversed");
                state = State.START_SHOOTER;
            }

            break;

        case START_SHOOTER:

            double spinupTime;

            if (!tooClose) {
                atArmAngle = driveArmToCalculatedAngle();
                spinupTime = 0.85;
            }
            else {
                atArmAngle = true;
                spinupTime = 0.5;
            }

            armSubsystem.setIntakeSpeed(0);

            double shooterSpeed = 0.75;
            if (lastDistanceToTarget >= 3) {
                shooterSpeed = 0.85;
            }
            armSubsystem.setShooterSpeed(shooterSpeed);

            // Wait for the shooter to get up to speed and the arm to get into position
            if (isStateTimeoutExceeded(spinupTime) && atArmAngle) {
                StringBuilder sb = new StringBuilder("Shooter up to speed & arm in position.");
                    sb.append(" TopShooter ")
                    .append(String.format("%.2f", armSubsystem.getTopShooterEncoderSpeed()))
                    .append(" BottomShooter ")
                    .append(String.format("%.2f", armSubsystem.getBottomShooterEncoderSpeed()))
                    .append(" Link ").append(armSubsystem.getLinkAngle()).append("deg")
                    .append(" Aim ").append(armSubsystem.getAimAngle()).append("deg")
                    .append(" DistanceToTarget ").append(lastDistanceToTarget);
                logStateTransition("Start Shooter -> Shoot", sb.toString());
                state = State.START_FEEDER;
            }

            break;

        case START_FEEDER:

            if (!tooClose) {
                driveArmToCalculatedAngle();
            }
            armSubsystem.setIntakeSpeed(1);

            if (isStateTimeoutExceeded(.5)) {
                logStateTransition("Shoot -> Finished", "Shot fired");
                state = State.FINISHED;
            }
            break;

        case FINISHED:

            break;

        }
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        lighting.removePattern(Shooting.class);

        armSubsystem.setAimPivotSpeed(0);
        armSubsystem.setLinkPivotSpeed(0);

        armSubsystem.setIntakeSpeed(0);
        armSubsystem.setShooterSpeed(0);

        logCommandEnd(interrupted);

        if (!interrupted) {
            if (DriverStation.isTeleop()) {
                CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
            }
        }
    }

}