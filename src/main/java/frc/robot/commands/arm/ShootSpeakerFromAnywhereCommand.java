package frc.robot.commands.arm;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Shooting;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.SpeakerShooterPolynomialAngleCalc;

/**
 * Move arm to speaker shoot pose
 * Set shooter speed (distance based)
 */
public class ShootSpeakerFromAnywhereCommand extends ArmBaseCommand {

    private enum State {
        START_SHOOTER_UNLOCK_ARM, DRIVE_ARM_TO_POSITION, START_FEEDER, FINISHED
    };

    private SwerveSubsystem     swerve;
    private LightingSubsystem   lighting;

    private State               state                = State.START_SHOOTER_UNLOCK_ARM;
    double                      intakeStartPosition  = 0;
    private double              lastDistanceToTarget = -1310;
    private boolean             tooClose             = false;
    private double              shooterStartTime     = 0;
    private boolean             rotateToTarget       = false;
    private long                shooterSpinupTime    = 0;
    private boolean             armInCompactAtInit   = false;
    private boolean             alignedAtInit        = false;
    private boolean             forwards;
    int                         alignedCount         = 0;

    private Constants.BotTarget botTarget;

    public ShootSpeakerFromAnywhereCommand(ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem,
        LightingSubsystem lighting) {
        this(armSubsystem, swerveSubsystem, lighting, false);
    }

    public ShootSpeakerFromAnywhereCommand(ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem,
        LightingSubsystem lighting, boolean rotateToTargetWhenNotTooClose) {
        super(armSubsystem);
        this.swerve         = swerveSubsystem;
        this.lighting       = lighting;
        this.rotateToTarget = rotateToTargetWhenNotTooClose;

        if (rotateToTarget) {
            addRequirements(swerve);
        }
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
            forwards  = false;
        }
        else {
            botTarget = Constants.BotTarget.RED_SPEAKER;
            forwards  = true;
        }

        // Use standard Shoot if we're close enough to the speaker
        if (getDistanceToTarget() < SpeakerShooterPolynomialAngleCalc.MIN_DISTANCE) {
            tooClose          = true;
            shooterSpinupTime = 800;
        }
        else {
            shooterSpinupTime = 850;
        }

        armInCompactAtInit  = isAtArmPosition(ArmConstants.COMPACT_ARM_POSITION, 2);
        alignedAtInit       = isAligned();
        intakeStartPosition = armSubsystem.getIntakePosition();

        lighting.addPattern(SIGNAL, Shooting.getInstance());
        logCommandStart();
    }

    /**
     * Get the distance to the target from current bot position. Store in class variable for
     * easy reference.
     * 
     * @return distance to target in meters
     */
    private double getDistanceToTarget() {
        Pose2d botPose          = swerve.getPose();
        double distanceToTarget = botPose.getTranslation().getDistance(botTarget.getLocation().toTranslation2d());
        lastDistanceToTarget = distanceToTarget;
        return distanceToTarget;
    }

    /**
     * Drive the arm to the calculated angle for the current distance to the target.
     * 
     * @return if arm is at desired position
     */
    private boolean driveArmToCalculatedAngle() {
        // Drive to the arm position at the same time
        double                linkAngle        = ArmConstants.SHOOT_SPEAKER_PODIUM_ARM_POSITION.linkAngle;
        double                distanceToTarget = getDistanceToTarget();
        double                aimAngle         = SpeakerShooterPolynomialAngleCalc.calculateAimAngle(distanceToTarget);
        Constants.ArmPosition armPositionNew   = new Constants.ArmPosition(linkAngle, aimAngle);

        return driveToArmPosition(armPositionNew, 2, 2);
    }

    /**
     * Set the shooter speed based on the distance to the target and records the 1st time the
     * shooter was started.
     * 
     * @param distance distance to target in meters
     */
    private void setShoooterByDistance(double distance) {

        if (distance >= 3) {
            armSubsystem.setShooterSpeed(0.85);
        }
        else {
            armSubsystem.setShooterSpeed(0.8);
        }

        if (shooterStartTime == 0) {
            shooterStartTime = System.currentTimeMillis();
        }
    }

    /**
     * Rotate the bot to face the target. Will perform checks to ensure we're not too close,
     * rotation is requested, and we're not already aligned.
     */
    public void rotateBotToTarget() {

        // If we are too close, don't rotate.
        // If it wasn't asked for, don't rotate.
        // If we're already aligned to target at start of command, don't rotate.
        if (tooClose || !rotateToTarget || alignedAtInit) {
            return;
        }

        // Do the rotate!
        Rotation2d heading    = swerve.getHeadingToFieldPosition(botTarget.getLocation().toTranslation2d())
            .plus(Rotation2d.fromDegrees(180 * (forwards ? 0 : 1)));
        Pose2d     targetPose = new Pose2d(swerve.getPose().getTranslation(), heading);
        swerve.driveToFieldPose(targetPose, MAX_TRANSLATION_SPEED_MPS);
    }

    /**
     * Check if the bot is aligned to the target. Will increment a counter if aligned.
     * 
     * @return if bot is currently aligned to target
     */
    private boolean isAligned() {
        Rotation2d heading = swerve.getHeadingToFieldPosition(botTarget.getLocation().toTranslation2d())
            .plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));
        boolean    aligned = swerve.isCloseEnough(heading);

        if (aligned) {
            alignedCount++;
        }
        else {
            alignedCount = 0;
        }

        return aligned;
    }

    @Override
    public void execute() {

        final boolean atArmAngle;

        switch (state) {

        case START_SHOOTER_UNLOCK_ARM:

            // Start rotation to target if conditions permit
            rotateBotToTarget();

            // Start the shooter
            setShoooterByDistance(getDistanceToTarget());

            if (!tooClose && armInCompactAtInit) {
                // Run the link motor back (up) for .15 seconds to unlock the arm
                armSubsystem.setLinkPivotSpeed(.3);
                armSubsystem.setAimPivotSpeed(0);

                if (isStateTimeoutExceeded(.2)) {
                    logStateTransition(State.DRIVE_ARM_TO_POSITION.name(), "Shooter Started. Arm Unlocked.");
                    state = State.DRIVE_ARM_TO_POSITION;
                }
            }
            else {
                logStateTransition(State.DRIVE_ARM_TO_POSITION.name(), "Shooter Started. Unlocked Already. Move Arm Now");
                state = State.DRIVE_ARM_TO_POSITION;
            }

            break;

        case DRIVE_ARM_TO_POSITION:

            // Only drive arm if we're not too close, otherwise compact position is good.
            if (!tooClose) {
                atArmAngle = driveArmToCalculatedAngle();
            }
            else {
                atArmAngle = true;
            }

            // Continue rotation to target if conditions permit
            rotateBotToTarget();

            // Call this again to make sure the shooter is up to speed for the right distance
            setShoooterByDistance(lastDistanceToTarget);

            // Wait for the shooter to get up to speed and the arm to get into position and rotation
            // to be complete
            if (((System.currentTimeMillis() - shooterStartTime) > shooterSpinupTime)
                && atArmAngle
                && (!rotateToTarget || tooClose || alignedCount >= 10)) {

                swerve.stop();
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

            if (isStateTimeoutExceeded(.125)) {
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

        swerve.stop();

        logCommandEnd(interrupted);

        if (!interrupted) {
            if (DriverStation.isTeleop()) {
                CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
            }
        }
    }

}