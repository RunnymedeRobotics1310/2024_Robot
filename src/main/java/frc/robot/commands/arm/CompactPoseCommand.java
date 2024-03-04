package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Move arm to speaker shoot pose
// Set shooter speed (distance based)
public class CompactPoseCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_OVER_BUMPER, MOVE_TO_UNLOCK, MOVE_TO_COMPACT, LOCK, LOCKED
    };

    private State state = State.MOVE_TO_OVER_BUMPER;

    public CompactPoseCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();

        if (armSubsystem.getAimAngle() > ArmConstants.OVER_BUMPER_POSITION.aimAngle) {
            state = State.MOVE_TO_OVER_BUMPER;
        }
        else {
            state = State.MOVE_TO_UNLOCK;
        }

        // Stop all arm motors to turn off the shooter and intake.
        armSubsystem.stop();
    }

    @Override
    public void execute() {

        boolean atArmAngle = false;

        switch (state) {

        case MOVE_TO_OVER_BUMPER:

            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.OVER_BUMPER_POSITION, 5);

            // If past the bumper danger, move to the intake position.
            if (atArmAngle) {
                logStateTransition("Move to unlock", "Arm over bumper");
                state = State.MOVE_TO_UNLOCK;
            }

            break;


        case MOVE_TO_UNLOCK:

            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.UNLOCK_POSITION, 5);

            // If past the bumper danger, move to the intake position.
            if (atArmAngle) {
                logStateTransition("Move to compact", "Arm at unlock position");
                state = State.MOVE_TO_UNLOCK;
            }

            break;


        case MOVE_TO_COMPACT:

            // Move to the requested angle with a tolerance of 2 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.COMPACT_ARM_POSITION, 2);

            // If past the bumper danger, move to the intake position.
            if (atArmAngle) {
                logStateTransition("Lock arm", "Arm at compact position");
                state = State.LOCK;
            }

            break;

        case LOCK:

            // Drive link down for .2 seconds
            armSubsystem.setLinkPivotSpeed(-.1);

            // If past the bumper danger, move to the intake position.
            if (this.isStateTimeoutExceeded(.2)) {

                armSubsystem.setLinkPivotSpeed(0);

                logStateTransition("Locked", "Arm locked");
                state = State.LOCKED;
            }

            break;

        case LOCKED:
            // Nothing to do here
            break;
        }
    }

    @Override
    public boolean isFinished() {

        // If the arm is in the locked position
        if (state == State.LOCKED) {
            setFinishReason("Arm Locked in Compact Position");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();

        logCommandEnd(interrupted);

        if (!interrupted) {
            CommandScheduler.getInstance().schedule(new CompactPoseCommand(armSubsystem));
        }
    }


}