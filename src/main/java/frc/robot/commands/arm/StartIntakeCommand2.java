package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Start Intake
// Move Aim/Arm
public class StartIntakeCommand2 extends ArmBaseCommand {

    private enum State {
        MOVE_TO_UNLOCK, EXTEND_AIM, EXTEND_BOTH, MOVE_TO_INTAKE, FINISHED
    };

    private State state = State.MOVE_TO_UNLOCK;

    public StartIntakeCommand2(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void initialize() {

        // If there is a note inside the robot, then do not start this command
        if (armSubsystem.isNoteDetected()) {
            log("Note detected in robot. StartIntakeCommand cancelled");
            return;
        }

        // If the arm is at the resting position, go to the unlock position first
        // If the aim is inside the bumper area, then move to over bumper first
        // else just go to the intake position
        if (armSubsystem.getAimAngle() < 42.6) {
            state = State.MOVE_TO_UNLOCK;
            logStateTransition("Start -> Unlock", "Arm Locked, aim angle " + armSubsystem.getAimAngle());
        }
        else if (armSubsystem.getAimAngle() < ArmConstants.OVER_BUMPER_POSITION.aimAngle) {
            state = State.EXTEND_BOTH;
            logStateTransition("Start -> Extend Both", "Arm over bumper, aim angle " + armSubsystem.getAimAngle());
        }

        logCommandStart(state.name());

    }

    @Override
    public void execute() {

        // If there is a note detected, then there is nothing to do
        if (armSubsystem.isNoteDetected()) {
            return;
        }

        boolean atArmPosition;

        switch (state) {

        case MOVE_TO_UNLOCK:

            // Run the link motor back (up) for .15 seconds to unlock the arm
            armSubsystem.setLinkPivotSpeed(.3);
            armSubsystem.setAimPivotSpeed(0);

            if (isStateTimeoutExceeded(.2)) {
                logStateTransition("Unlock -> Extend Aim", "Arm Unlocked");
                state = State.EXTEND_AIM;
            }

            break;

        case EXTEND_AIM:

            // Start by extending the aim
            armSubsystem.setLinkPivotSpeed(0);
            armSubsystem.setAimPivotSpeed(.3);

            // Once the arm has extended by 15 deg, then start a sychro movement
            if (armSubsystem.getAimAngle() > ArmConstants.COMPACT_ARM_POSITION.aimAngle + 5) {
                logStateTransition("Extend Aim -> Extend Both", "Aim at " + armSubsystem.getAimAngle());
                state = State.EXTEND_BOTH;
            }

            break;

        case EXTEND_BOTH:

            // Start by extending the aim
            armSubsystem.setLinkPivotSpeed(-1);
            armSubsystem.setAimPivotSpeed(.3);

            // Once the aim has reached the target, then stop the aim.
            if (armSubsystem.getAimAngle() > ArmConstants.INTAKE_ARM_POSITION.aimAngle + 0) {

                logStateTransition("Extend Both -> Move to intake", "Aim at " + armSubsystem.getAimAngle());
                state = State.MOVE_TO_INTAKE;

            }

            break;

        case MOVE_TO_INTAKE:

            // Start by extending the aim
            armSubsystem.setLinkPivotSpeed(-1);
            armSubsystem.setAimPivotSpeed(0);

            // Rest the aim on the hard stop
            if (armSubsystem.isLinkAtLowerLimit()) {
                logStateTransition("Move to Intake -> Finished", "At intake position");
                state = State.FINISHED;
            }

            break;

        case FINISHED:
            break;
        }

    }

    @Override
    public boolean isFinished() {

        // If the arm is in position, then this command ends
        if (state == State.FINISHED) {
            setFinishReason("At Intake Position");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();

        logCommandEnd(interrupted);
    }

}