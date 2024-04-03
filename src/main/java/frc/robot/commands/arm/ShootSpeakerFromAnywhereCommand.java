package frc.robot.commands.arm;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.swervedrive.SwerveUtils;
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
        MOVE_TO_UNLOCK, START_SHOOTER, START_FEEDER, FINISHED
    };

    private SwerveSubsystem       swerveSubsystem;
    private LightingSubsystem     lighting;

    private State                 state                  = State.MOVE_TO_UNLOCK;
    double                        intakeStartPosition    = 0;
    private Pose2d                robotPose              = null;
    private double                lastDistanceToTarget   = -1310;
    private boolean               tooClose               = false;
    private long                  shooterStartTimeNanos  = 0;
    private long                  shooterSpinUpTimeNanos = 0;
    private double                shooterSpeedRequired   = 0.0;
    private long                  armMoveStartTimeNanos  = 0;
    private Constants.ArmPosition shotArmPosition        = null;

    private Constants.BotTarget   botTarget;


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

        updateShooterConfig();

        // Use standard Shoot if we're close enough to the speaker
        if (tooClose) {
            state = State.START_SHOOTER;
        }
        else if (isAtArmPosition(ArmConstants.COMPACT_ARM_POSITION, 2)) {
            state = State.MOVE_TO_UNLOCK;
        }
        else {
            state = State.START_SHOOTER;
        }

        intakeStartPosition = armSubsystem.getIntakePosition();
    }


    private boolean updateShooterConfig() {
        final boolean changed;
        if (this.robotPose == null) {
            this.robotPose = swerveSubsystem.getPose();
            changed        = true;
        }
        else {
            Pose2d poseNow = swerveSubsystem.getPose();
            if (SwerveUtils.isCloseEnough(poseNow.getTranslation(), robotPose.getTranslation())
                && SwerveUtils.isCloseEnough(poseNow.getRotation(), robotPose.getRotation(), ROTATION_TOLERANCE)) {
                changed = false;
            }
            else {
                // NOTEoriousPID moved!
                this.robotPose = poseNow;
                changed        = true;
            }
        }

        if (changed) {
            lastDistanceToTarget = robotPose.getTranslation().getDistance(botTarget.getLocation().toTranslation2d());
            if (lastDistanceToTarget >= 3.9) {
                this.shooterSpeedRequired   = 0.95;
                this.shooterSpinUpTimeNanos = 1100 * 1000 * 1000;
            }
            else if (lastDistanceToTarget >= 3) {
                this.shooterSpeedRequired   = 0.85;
                this.shooterSpinUpTimeNanos = 850 * 1000 * 1000;
            }
            else {
                this.shooterSpeedRequired   = 0.8;
                this.shooterSpinUpTimeNanos = 850 * 1000 * 1000;
            }

            if (lastDistanceToTarget < 1.6) {
                tooClose             = true;
                // we're too close! shoot from compact
                this.shotArmPosition = ArmConstants.COMPACT_ARM_POSITION;
            }
            else {
                tooClose             = false;
                // we're back a bit - shoot from a calculated angle
                this.shotArmPosition = new Constants.ArmPosition(ArmConstants.SHOOT_SPEAKER_PODIUM_ARM_POSITION.linkAngle,
                    SpeakerShooterPolynomialAngleCalc.calculateAimAngle(lastDistanceToTarget));
            }
        }

        return changed;
    }

    private boolean driveArmToCalculatedAngle() {
        if (armMoveStartTimeNanos == 0) {
            armMoveStartTimeNanos = System.nanoTime();
        }
        boolean inPosition = driveToArmPosition(this.shotArmPosition, 2, 2);
        if (tooClose && inPosition) {
            // don't drive motors if we're in compact. This SHOULD already be in the subsystem
            armSubsystem.setLinkPivotSpeed(0);
            armSubsystem.setAimPivotSpeed(0);
        }
        return inPosition;
    }

    private void startShooter() {
        armSubsystem.setIntakeSpeed(0);
        if (armSubsystem.getIntakeEncoderSpeed() < 0.01) {
            armSubsystem.setShooterSpeed(this.shooterSpeedRequired);
            if (shooterStartTimeNanos == 0) {
                shooterStartTimeNanos = System.nanoTime();
            }
        }
        {
            log("Cannot start shooter. Intake is still moving.");
        }
    }

    @Override
    public void execute() {

        boolean botMoved = updateShooterConfig();
        if (botMoved) {
            // if the bot moved, give the robot extra time to get into position
            this.armMoveStartTimeNanos = 0;
            this.shooterStartTimeNanos = 0;
            // if we suddenly became too close, start the shooter.
            if (tooClose) {
                logStateTransition("START_SHOOTER", "Bot moved - now too close. Start shooter.");
                state = State.START_SHOOTER;
            }
        }

        switch (state) {

        case MOVE_TO_UNLOCK:

            // Start the shooter
            startShooter();

            // Run the link motor back (up) for .15 seconds to unlock the arm
            armSubsystem.setLinkPivotSpeed(.3);
            armSubsystem.setAimPivotSpeed(0);

            if (isStateTimeoutExceeded(.2)) {
                logStateTransition("Unlock -> Move To Speaker", "Arm Unlocked");
                state = State.START_SHOOTER;
            }

            break;

        case START_SHOOTER:

            startShooter();

            long now = System.nanoTime();
            boolean atArmAngle = driveArmToCalculatedAngle();
            boolean armTimeout = now > (armMoveStartTimeNanos + (2000 * 1000 * 1000));
            boolean shooterReady = now > (shooterStartTimeNanos + shooterSpinUpTimeNanos);
            boolean armReady = tooClose || atArmAngle || armTimeout;

            // Wait for the shooter to get up to speed and the arm to get into position
            if (shooterReady && armReady) {
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

            // This is to KEEP the arm at the desired angle. If we are in this state
            // the arm already got to the desired angle.
            driveArmToCalculatedAngle();

            // feed the note into the shooter
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
