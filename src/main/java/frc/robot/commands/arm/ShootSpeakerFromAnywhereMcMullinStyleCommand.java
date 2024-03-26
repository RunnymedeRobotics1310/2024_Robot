package frc.robot.commands.arm;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

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

/**
 * Move arm to speaker shoot pose
 * Set shooter speed (distance based)
 */
public class ShootSpeakerFromAnywhereMcMullinStyleCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_UNLOCK, REVERSE_NOTE, START_SHOOTER, START_FEEDER, FINISHED
    };

    private SwerveSubsystem     swerveSubsystem;
    private LightingSubsystem   lighting;

    private State               state               = State.MOVE_TO_UNLOCK;
    double                      intakeStartPosition = 0;
    long                        shooterStartTime    = 0;
    private Constants.BotTarget botTarget;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Testing");
    NetworkTableEntry aimMotorSpeed = table.getEntry("aimMotorSpeed");
    NetworkTableEntry linkMotorSpeed = table.getEntry("linkMotorSpeed");
    NetworkTableEntry aimDuration = table.getEntry("aimDuration");
    NetworkTableEntry linkDuration = table.getEntry("linkDuration");

    public ShootSpeakerFromAnywhereMcMullinStyleCommand(ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem,
                                                        LightingSubsystem lighting) {
        super(armSubsystem);
        this.swerveSubsystem     = swerveSubsystem;
        this.lighting            = lighting;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        lighting.addPattern(SIGNAL, Shooting.getInstance());
        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            log("No note detected in robot. AimSpeakerCommand cancelled");
            state = State.FINISHED;
            return;
        }

        logCommandStart();

        if (isAtArmPosition(ArmConstants.COMPACT_ARM_POSITION, 2)) {
            state = State.MOVE_TO_UNLOCK;
        }
        else {
            state = State.REVERSE_NOTE;
        }

        intakeStartPosition = armSubsystem.getIntakePosition();

        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            botTarget = Constants.BotTarget.BLUE_SPEAKER;
        }
        else {
            botTarget = Constants.BotTarget.RED_SPEAKER;
        }
    }

    /**
     * Calculates aim angle based on quadratic equation fit for the following data:
     * - 1.5 meter distance, angle 40 (by paper measurement, but feels odd. let's try 43)
     * - 3 meters distance, angle 42
     * - 5 meters distance, angle 53
     *
     * @param distance Distance in meters to target
     * @return aim angle in degrees
     */
    private double calculateAimAngle(double distance) {
        // Coefficients from the quadratic equation fit
        double a = 0.85714286;
        double b = -1.85714286;
        double c = 40.85714286;

        // Calculate the angle based on the distance
        return a * Math.pow(distance, 2) + b * distance + c;
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

            // Drive to the arm position at the same time
            double linkAngle = ArmConstants.SHOOT_SPEAKER_PODIUM_ARM_POSITION.linkAngle;

            double aimMotorSpeed = this.aimMotorSpeed.getDouble(0);
            double linkMotorSpeed = this.linkMotorSpeed.getDouble(0);
            double aimDuration = this.aimDuration.getDouble(0);
            double linkDuration = this.aimDuration.getDouble(0);

            Pose2d botPose = swerveSubsystem.getPose();
            double distanceToTarget = botPose.getTranslation().getDistance(botTarget.getLocation().toTranslation2d());
            // double aimAngle = calculateAimAngle(distanceToTarget);

            if (isStateTimeoutExceeded(linkDuration)) {
                double holdSpeed = calcLinkHold(armSubsystem.getAimAngle(), armSubsystem.getLinkAngle());
                armSubsystem.setLinkPivotSpeed(holdSpeed);
            }
            else {
                armSubsystem.setLinkPivotSpeed(linkMotorSpeed);
            }

            if (isStateTimeoutExceeded(aimDuration)) {
                double holdSpeed = calcAimHold(armSubsystem.getAimAngle(), armSubsystem.getLinkAngle());
                armSubsystem.setAimPivotSpeed(holdSpeed);
            }
            else {
                armSubsystem.setAimPivotSpeed(aimMotorSpeed);
            }

            armSubsystem.setIntakeSpeed(0);

            double shooterSpeed;
            if (distanceToTarget < 3.5) {
                shooterSpeed = 0.75;
            }
            else {
                shooterSpeed = 1;
            }
            armSubsystem.setShooterSpeed(shooterSpeed);

            // Wait for the shooter to get up to speed and the arm to get into position
            if (isStateTimeoutExceeded(linkDuration + .25) && isStateTimeoutExceeded(linkDuration+ .25)) {
                logStateTransition("Start Shooter -> Shoot", "Shooter up to speed " + armSubsystem.getBottomShooterEncoderSpeed()
                + ",DistanceToTarget["+distanceToTarget+"],encoderAimAngle["+armSubsystem.getAimAngle()+"],encoderLinkAngle["+armSubsystem.getLinkAngle()+"]");
                state = State.START_FEEDER;
            }

            break;

        case START_FEEDER:

            // Continue to drive to the arm position while shooting
            double linkHoldSpeed = calcLinkHold(armSubsystem.getAimAngle(), armSubsystem.getLinkAngle());
            armSubsystem.setLinkPivotSpeed(linkHoldSpeed);
            double aimHoldSpeed = calcAimHold(armSubsystem.getAimAngle(), armSubsystem.getLinkAngle());
            armSubsystem.setAimPivotSpeed(aimHoldSpeed);

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

        if (state == State.FINISHED) {
            return true;
        }
        else {
            return false;
        }
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