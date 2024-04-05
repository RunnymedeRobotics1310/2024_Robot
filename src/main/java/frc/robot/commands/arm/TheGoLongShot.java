package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.ArmConstants.LONG_SHOT_ARM_POSITION;
import static frc.robot.Constants.FieldConstants.FIELD_EXTENT_METRES_X;
import static frc.robot.Constants.FieldConstants.WING_LENGTH_METRES;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;
import static frc.robot.commands.arm.TheGoLongShot.State.*;

public class TheGoLongShot extends ArmBaseCommand {
    enum State {
        MOVE_TO_UNLOCK, START_SHOOTER, START_FEEDER, FINISHED
    };

    private SwerveSubsystem drive;

    private State           state                     = MOVE_TO_UNLOCK;
    private static long     SHOOTER_SPINUP_TIME_NANOS = 1200 * 1000 * 1000;
    private long            shooterStartTimeNanos     = 0;


    public TheGoLongShot(ArmSubsystem armSubsystem, SwerveSubsystem drive) {
        super(armSubsystem);
        this.drive = drive;
    }


    @Override
    public void initialize() {
        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            log("No note detected in robot. Cancelling.");
            state = FINISHED;
            return;
        }

        Pose2d pose = drive.getPose();
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            if (pose.getX() > (FIELD_EXTENT_METRES_X - WING_LENGTH_METRES)) {
                log("On blue alliance but bot is inside the red wing. Cancelling.");
                state = FINISHED;
                return;
            }
        }
        else {
            if (pose.getX() < (WING_LENGTH_METRES)) {
                log("On red alliance but bot is inside the blue wing. Cancelling.");
                state = FINISHED;
                return;
            }
        }

        logCommandStart();
        shooterStartTimeNanos = 0;
        state                 = MOVE_TO_UNLOCK;
    }


    @Override
    public void execute() {

        final boolean atArmAngle;

        switch (state) {

        case MOVE_TO_UNLOCK:

            // Start the shooter
            if (shooterStartTimeNanos == 0) {
                shooterStartTimeNanos = System.nanoTime();
            }
            armSubsystem.setShooterSpeed(0.95);
            armSubsystem.setIntakeSpeed(0);

            // Run the link motor back (up) for .15 seconds to unlock the arm
            armSubsystem.setLinkPivotSpeed(.3);
            armSubsystem.setAimPivotSpeed(0);

            if (isStateTimeoutExceeded(.2)) {
                logStateTransition("Unlock -> Move To Speaker", "Arm Unlocked");
                state = START_SHOOTER;
            }

            break;

        case START_SHOOTER:

            atArmAngle = driveToArmPosition(LONG_SHOT_ARM_POSITION, 3, 3);
            armSubsystem.setShooterSpeed(0.95);
            armSubsystem.setIntakeSpeed(0);

            long elapsedNanos = System.nanoTime() - shooterStartTimeNanos;
            if (atArmAngle && (elapsedNanos > SHOOTER_SPINUP_TIME_NANOS)) {
                logStateTransition("Start Shooter -> Shoot", "Shot fired");
                state = START_FEEDER;
            }

            break;

        case START_FEEDER:

            armSubsystem.setIntakeSpeed(1);

            if (isStateTimeoutExceeded(.25)) {
                logStateTransition("Shoot -> Finished", "Shot fired");
                state = FINISHED;
            }
            break;

        case FINISHED:

            armSubsystem.setAimPivotSpeed(0);
            armSubsystem.setLinkPivotSpeed(0);

            armSubsystem.setIntakeSpeed(0);
            armSubsystem.setShooterSpeed(0);

            break;

        }
    }

    @Override
    public boolean isFinished() {
        return state == FINISHED;
    }

    @Override
    public void end(boolean interrupted) {

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
