package frc.robot.commands.arm;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

// Shoot. That's it.
public class ReverseNoteCommand extends LoggingCommand {

    private enum State {
        ENSURE_INTAKE_STOPPED, REVERSE_NOTE, FINISHED
    };

    private State              state               = State.ENSURE_INTAKE_STOPPED;

    private double             intakeStartPosition = 0;

    private final ArmSubsystem armSubsystem;

    public ReverseNoteCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {

//        if (!armSubsystem.isNoteDetected()) {
//            logCommandStart("No note detected in robot. ReverseNoteCommand cancelled");
//            state = State.FINISHED;
//            return;
//
//        }
        state = State.ENSURE_INTAKE_STOPPED;
        logCommandStart("Intake Speed: " + armSubsystem.getIntakeEncoderSpeed());
    }

    @Override
    public void execute() {

        switch (state) {

        case ENSURE_INTAKE_STOPPED:
            armSubsystem.setIntakeSpeed(0);

            // TODO: Validate the 10 constant on robot
            if (armSubsystem.getIntakeEncoderSpeed() <= 10) {
                intakeStartPosition = armSubsystem.getIntakePosition();
                logStateTransition(State.REVERSE_NOTE.name(), "Intake stopped.  Time to reverse");
                state = State.REVERSE_NOTE;
            }
            break;

        case REVERSE_NOTE:

            armSubsystem.setShooterSpeed(-0.1);
            armSubsystem.setIntakeSpeed(-0.3);

            // Reverse the note for a number of rotations
            if (Math.abs(armSubsystem.getIntakePosition() - intakeStartPosition) > 1.5) {
                armSubsystem.setIntakeSpeed(0);
                armSubsystem.setShooterSpeed(0);
                logStateTransition(State.FINISHED.name(), "Shooter Reversed");
                state = State.FINISHED;
            }

            break;

        default:
            break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setIntakeSpeed(0);
        armSubsystem.setShooterSpeed(0);
        logCommandEnd(interrupted);
    }

}
