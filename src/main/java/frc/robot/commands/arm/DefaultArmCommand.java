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

    private static final long            NOTE_DETECT_TIMEOUT = 10000;

    private static final long            NOTE_FORWARD_TIME   = 300;


    private enum State {
        WAIT_FOR_NOTE, NOTE_DETECTED, REVERSE_NOTE, FORWARD_NOTE, NOTE_READY, KILLED
    };

    private State state                = State.WAIT_FOR_NOTE;

    private long  lastNoteDetectedTime = 0;

    private long  noteForwardStartTime = 0;


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

        switch (state) {

        case WAIT_FOR_NOTE:
            // Notes should not be detected in this state but as a failsafe this will prevent the
            // state machine from getting stuck
            if (armSubsystem.isNoteDetected()) {
                lastNoteDetectedTime = System.currentTimeMillis();
                System.out.println("safety code activated, switched to note detect");
                state = State.NOTE_DETECTED;
            }
            // This is the normal case
            if (jackmanVisionSubsystem.isNoteClose()) {
                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_INTAKE_SPEED);
                lastNoteDetectedTime = System.currentTimeMillis();
                System.out.println("Switched to note detect");
                state = State.NOTE_DETECTED;
            }
            break;

        case NOTE_DETECTED:
            if (armSubsystem.isNoteDetected()) {
                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_NOTE_REVERSAL_REVERSE_SPEED);
                System.out.println("Switched to Reverse Note");
                state = State.REVERSE_NOTE;
            }
            else if (jackmanVisionSubsystem.isNoteClose()) {
                lastNoteDetectedTime = System.currentTimeMillis();
            }
            else if (System.currentTimeMillis() - lastNoteDetectedTime >= NOTE_DETECT_TIMEOUT) {
                armSubsystem.setIntakeSpeed(0);
                System.out.println("timedout, switched to wait for note");
                state = State.WAIT_FOR_NOTE;
            }
            break;

        case REVERSE_NOTE:
            if (!armSubsystem.isNoteDetected()) {
                noteForwardStartTime = System.currentTimeMillis();
                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_NOTE_REVERSAL_FORWARD_SPEED);
                System.out.println("Switched to forward Note");
                state = State.FORWARD_NOTE;
            }
            break;

        case FORWARD_NOTE:
            if ((System.currentTimeMillis() - noteForwardStartTime >= NOTE_FORWARD_TIME)) {
                armSubsystem.setIntakeSpeed(0);
                System.out.println("Switched to Note ready");
                state = State.NOTE_READY;
            }
            break;

        case NOTE_READY:
            if (!armSubsystem.isNoteDetected()) {
                System.out.println("Switched to wait for note from note ready");
                state = State.WAIT_FOR_NOTE;
            }
            break;

        case KILLED:
            if (armSubsystem.isNoteDetected()) {
                state = State.NOTE_READY;
            }
            else {
                state = State.WAIT_FOR_NOTE;
            }
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
        armSubsystem.setIntakeSpeed(0);
        state = State.KILLED;
    }

    private void setLinkMotorSpeed(double speed) {
        armSubsystem.setLinkPivotSpeed(speed);
    }

    private void setAimMotorSpeed(double speed) {
        armSubsystem.setAimPivotSpeed(speed);
    }

}