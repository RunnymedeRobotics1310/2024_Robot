package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Move arm to amp shoot pose
// Set shooter speed
public class AimAmpCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_UNLOCK, MOVE_TO_AMP, FINISHED
    };

    private State state = State.MOVE_TO_AMP;

    public AimAmpCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {

        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            log(" No note detected in robot. AimAmpCommand cancelled");
            state = State.FINISHED;
            return;
        }

        logCommandStart();

        if (isAtArmPosition(ArmConstants.COMPACT_ARM_POSITION, 2)) {
            state = State.MOVE_TO_UNLOCK;
        }
        else {
            state = State.MOVE_TO_AMP;
        }
    }

    @Override
    public void execute() {

        boolean atArmAngle = false;

        switch (state) {

        case MOVE_TO_UNLOCK:

            // Run the link motor back (up) for .15 seconds to unlock the arm
            armSubsystem.setLinkPivotSpeed(.3);
            armSubsystem.setAimPivotSpeed(0);

            if (isStateTimeoutExceeded(.2)) {
                logStateTransition("Unlock -> Move To Amp", "Arm Unlocked");
                state = State.MOVE_TO_AMP;
            }

            break;

        case MOVE_TO_AMP:

            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.SHOOT_AMP_ARM_POSITION, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
                ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

        default:
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
        armSubsystem.setAimPivotSpeed(0);
        armSubsystem.setLinkPivotSpeed(0);
    }
}