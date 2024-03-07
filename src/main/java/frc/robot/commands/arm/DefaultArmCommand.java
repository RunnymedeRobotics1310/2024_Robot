package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

import static frc.robot.commands.operator.OperatorInput.Axis.Y;
import static frc.robot.commands.operator.OperatorInput.Stick.LEFT;
import static frc.robot.commands.operator.OperatorInput.Stick.RIGHT;

public class DefaultArmCommand extends LoggingCommand {

    private final ArmSubsystem           armSubsystem;
    private final JackmanVisionSubsystem jackmanVisionSubsystem;
    private final OperatorInput          operatorInput;

    private enum State {
        WAIT_FOR_NOTE, NOTE_DETECTED, NOTE_AQUIRED, REVERSE_NOTE, PLACE_NOTE_PROPER
    };

    private State state                = State.WAIT_FOR_NOTE;

    private long  lastNoteDetectedTime = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param armSubsystem The subsystem used by this command.
     */
    public DefaultArmCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackmanVisionSubsystem) {

        this.operatorInput          = operatorInput;
        this.armSubsystem           = armSubsystem;
        this.jackmanVisionSubsystem = jackmanVisionSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (!operatorInput.isShift()) {
            setLinkMotorSpeed(operatorInput.getOperatorControllerAxis(LEFT, Y) * 0.2);
            setAimMotorSpeed(operatorInput.getOperatorControllerAxis(RIGHT, Y) * 0.2);
        }
        else {
            setLinkMotorSpeed(0);
            setAimMotorSpeed(0);
        }

        runIntakeStateMachine();
    }

    private void runIntakeStateMachine() {

//        if (armSubsystem.isNoteDetected()) {
//            armSubsystem.setIntakeSpeed(0);
//        }
//        else {
//            if (jackmanVisionSubsystem.isNoteClose()) {
//                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_INTAKE_SPEED);
//            }
//            else {
//                armSubsystem.setIntakeSpeed(0);
//            }
//        }

        switch (state) {

        case WAIT_FOR_NOTE:
            if (armSubsystem.isNoteDetected()) {
                state = State.NOTE_DETECTED;
            }
            break;

        case NOTE_DETECTED:
            if (armSubsystem.isNoteDetected()) {
                state = State.NOTE_AQUIRED;
            }
            break;

        case NOTE_AQUIRED:
            if (armSubsystem.isNoteDetected()) {
                state = State.REVERSE_NOTE;
            }
            break;

        case REVERSE_NOTE:
            if (!armSubsystem.isNoteDetected()) {
                state = State.PLACE_NOTE_PROPER;
            }
            break;

        case PLACE_NOTE_PROPER:
            if (armSubsystem.isNoteDetected()) {
                state = State.NOTE_DETECTED;
            }
            break;

        default:
            break;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The default arm command never ends, but can be interrupted by other commands.
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

    private void setLinkMotorSpeed(double speed) {
        armSubsystem.setLinkPivotSpeed(speed);
    }

    private void setAimMotorSpeed(double speed) {
        armSubsystem.setAimPivotSpeed(speed);
    }

}