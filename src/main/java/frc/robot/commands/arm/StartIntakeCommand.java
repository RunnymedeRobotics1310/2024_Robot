package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Start Intake
// Move Aim/Arm
public class StartIntakeCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_UNLOCK, MOVE_TO_OVER_BUMPER, MOVE_TO_INTAKE
    };

    private State state = State.MOVE_TO_UNLOCK;

    public StartIntakeCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void initialize() {

        // If there is a note inside the robot, then do not start this command
        if (armSubsystem.isNoteDetected()) {
            System.out.println("Note detected in robot. StartIntakeCommand cancelled");
            return;
        }

        logCommandStart();

        // If the arm is at the resting position, go to the unlock position first
        // If the aim is inside the bumper area, then move to over bumper first
        // else just go to the intake position
        if (armSubsystem.getAimAngle() < 90) {
            state = State.MOVE_TO_UNLOCK;
        }
        else if (armSubsystem.getAimAngle() < ArmConstants.OVER_BUMPER_POSITION.aimAngle) {
            state = State.MOVE_TO_OVER_BUMPER;
        }
        else {
            state = State.MOVE_TO_INTAKE;
        }

    }

    @Override
    public void execute() {

        // If there is a note detected, then there is nothing to do
        if (armSubsystem.isNoteDetected()) {
            return;
        }

        boolean atArmPosition = false;

        switch (state) {

        case MOVE_TO_UNLOCK:

            // Move to the requested angle with a tolerance of 3 deg
            atArmPosition = this.driveToArmPosition(ArmConstants.UNLOCK_POSITION, 3);

            if (atArmPosition) {
                logStateTransition("Move to over bumper", "Arm Unlocked");
                state = State.MOVE_TO_OVER_BUMPER;
            }

            break;

        case MOVE_TO_OVER_BUMPER:

            // Move to the requested angle with a tolerance of 5 deg
            atArmPosition = this.driveToArmPosition(ArmConstants.OVER_BUMPER_POSITION, 5);

            // If past the bumper danger, move to the intake position.
            if (atArmPosition) {
                logStateTransition("Move to intake", "Arm over bumper");
                state = State.MOVE_TO_INTAKE;
            }

            break;

        case MOVE_TO_INTAKE:

            // Start the intake wheels
            armSubsystem.setIntakeSpeed(ArmConstants.INTAKE_INTAKE_SPEED);

            // Move to the requested angle with a tolerance of 2 deg
            this.driveToArmPosition(ArmConstants.INTAKE_ARM_POSITION, 2);

            break;

        }
    }

    @Override
    public boolean isFinished() {

        // If there is a note detected, then this command ends
        if (armSubsystem.isNoteDetected()) {
            setFinishReason("Note detected");
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