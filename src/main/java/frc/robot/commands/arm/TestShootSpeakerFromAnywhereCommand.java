package frc.robot.commands.arm;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RunnymedeUtils;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Shooting;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.SpeakerShooterPolynomialAngleCalc;

/**
 * Move arm to speaker shoot pose
 * Set shooter speed (distance based)
 */
public class TestShootSpeakerFromAnywhereCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_UNLOCK, START_SHOOTER, START_FEEDER, FINISHED
    };

    private SwerveSubsystem     swerveSubsystem;
    private LightingSubsystem   lighting;

    private State               state                              = State.MOVE_TO_UNLOCK;
    double                      intakeStartPosition                = 0;
    private double              lastDistanceToTarget               = -1310;
    private boolean             tooClose                           = false;
    private long                shooterStartTime                   = 0;
    private long                shooterStartTimeCurrrentTimeMillis = 0;
    private long                shooterSpinUpTime                  = 850;

    private Constants.BotTarget botTarget;

    NetworkTable                table                              = NetworkTableInstance.getDefault().getTable("Testing");
    NetworkTableEntry           shooterSpeedNT                     = table.getEntry("shooterSpeed");


    public TestShootSpeakerFromAnywhereCommand(ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem,
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

        shooterStartTime                   = 0;
        shooterStartTimeCurrrentTimeMillis = 0;

        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            botTarget = Constants.BotTarget.BLUE_SPEAKER;
        }
        else {
            botTarget = Constants.BotTarget.RED_SPEAKER;
        }

        lighting.addSignalPattern(Shooting.getInstance());

        logCommandStart();

        state               = State.MOVE_TO_UNLOCK;

        intakeStartPosition = armSubsystem.getIntakePosition();
    }

    private boolean driveArmToCalculatedAngle() {
        // Drive to the arm position at the same time
        double                linkAngle        = ArmConstants.SHOOT_SPEAKER_PODIUM_ARM_POSITION.linkAngle;
        double                distanceToTarget = 2.0;
        double                aimAngle         = SpeakerShooterPolynomialAngleCalc.calculateAimAngle(distanceToTarget);
        Constants.ArmPosition armPositionNew   = new Constants.ArmPosition(linkAngle, aimAngle);

        return driveToArmPosition(armPositionNew, 2, 2);
    }

    private double getDistanceToTarget() {
        Pose2d botPose          = swerveSubsystem.getPose();
        double distanceToTarget = botPose.getTranslation().getDistance(botTarget.getLocation().toTranslation2d());
        lastDistanceToTarget = distanceToTarget;
        return distanceToTarget;
    }

    private void setShoooter() {

        double shooterSpeed = shooterSpeedNT.getDouble(0.85);
        armSubsystem.setShooterSpeed(shooterSpeed);
        shooterSpinUpTime = 1500;
//        if (shooterSpeed > 0.85) {
//            shooterSpinUpTime = 1100;
//        }
//        else if (shooterSpeed > 0.80) {
//            shooterSpinUpTime = 850;
//        }
//        else {
//            shooterSpinUpTime = 850;
//        }

        if (shooterStartTime == 0) {
            shooterStartTime                   = RunnymedeUtils.relativeTimeMillis();
            shooterStartTimeCurrrentTimeMillis = System.currentTimeMillis();
        }
    }

    @Override
    public void execute() {

        final boolean atArmAngle;

        long          thisTime                  = RunnymedeUtils.relativeTimeMillis();
        long          thisTimeCurrentTimeMillis = System.currentTimeMillis();
        long          timeDiff                  = (thisTime - shooterStartTime)
            - (thisTimeCurrentTimeMillis - shooterStartTimeCurrrentTimeMillis);

        StringBuilder shootSb                   = new StringBuilder("SHOOTSTAT ");
        shootSb.append(" Power ")
            .append(shooterSpeedNT.getDouble(0.85))
            .append(" TopShooter ")
            .append(String.format("%.2f", armSubsystem.getTopShooterEncoderSpeed()))
            .append(" BottomShooter ")
            .append(String.format("%.2f", armSubsystem.getBottomShooterEncoderSpeed()))
            .append(" Link ").append(armSubsystem.getLinkAngle()).append("deg")
            .append(" Aim ").append(armSubsystem.getAimAngle()).append("deg")
            .append(" ShooterElasped ")
            .append(thisTime - shooterStartTime)
            .append(" StateElasped ")
            .append(getStateElapsedTime() * 1000)
            .append(" ElaspedDelta ")
            .append((getStateElapsedTime() * 1000) - (thisTime - shooterStartTime))
            .append(" CurrentTimeDelta ")
            .append(timeDiff)
            .append(" Distance ")
            .append(getDistanceToTarget());
        System.out.println(shootSb);

        switch (state) {

        case MOVE_TO_UNLOCK:

            // Start the shooter
            setShoooter();

            // Run the link motor back (up) for .15 seconds to unlock the arm
            armSubsystem.setLinkPivotSpeed(.3);
            armSubsystem.setAimPivotSpeed(0);

            if (isStateTimeoutExceeded(.2)) {
                logStateTransition("Unlock -> Move To Speaker", "Arm Unlocked");
                state = State.START_SHOOTER;
            }

            break;

        case START_SHOOTER:

            if (!tooClose) {
                atArmAngle = driveArmToCalculatedAngle();
            }
            else {
                atArmAngle = true;
            }

            armSubsystem.setIntakeSpeed(0);
            setShoooter();

            // Wait for the shooter to get up to speed and the arm to get into position
            if (((RunnymedeUtils.relativeTimeMillis() - shooterStartTime) > shooterSpinUpTime) && atArmAngle) {
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

            if (isStateTimeoutExceeded(.25)) {
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