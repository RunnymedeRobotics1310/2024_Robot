package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class IntakeCommand extends LoggingCommand {

    private final ArmSubsystem           armSubsystem;
    private final JackmanVisionSubsystem jackmanVisionSubsystem;

    private static final long            NOTE_DETECT_TIMEOUT = 10000;

    private static final long            NOTE_FORWARD_TIME   = 300;

    private static final long            ARM_UP_TIME         = 1000;



    private enum State {
        WAIT_FOR_NOTE, NOTE_DETECTED, RAISE_ARM, REVERSE_NOTE, FORWARD_NOTE, NOTE_READY, KILLED
    };

    private IntakeCommand.State state                = IntakeCommand.State.WAIT_FOR_NOTE;

    private long                lastNoteDetectedTime = 0;

    private long                noteForwardStartTime = 0;

    private long                armUpStartTime       = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param armSubsystem The subsystem used by this command.
     */
    public IntakeCommand(ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackmanVisionSubsystem) {

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

        switch (state) {

        case WAIT_FOR_NOTE:
            // Notes should not be detected in this state but as a failsafe this will prevent the
            // state machine from getting stuck
            if (armSubsystem.isNoteDetected()) {
                lastNoteDetectedTime = System.currentTimeMillis();
                System.out.println("safety code activated, switched to note detect");
                state = IntakeCommand.State.NOTE_DETECTED;
            }
            // This is the normal case
            if (jackmanVisionSubsystem.isNoteClose()) {
                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_INTAKE_SPEED);
                lastNoteDetectedTime = System.currentTimeMillis();
                System.out.println("Switched to note detect");
                state = IntakeCommand.State.NOTE_DETECTED;
            }
            break;

        case NOTE_DETECTED:
            if (armSubsystem.isNoteDetected()) {
                armUpStartTime = System.currentTimeMillis();
                armSubsystem.setAimPivotSpeed(0.4);
                System.out.println("Switched to Raise Arm");
                state = IntakeCommand.State.RAISE_ARM;
            }
            else if (jackmanVisionSubsystem.isNoteClose()) {
                lastNoteDetectedTime = System.currentTimeMillis();
            }
            else if (System.currentTimeMillis() - lastNoteDetectedTime >= NOTE_DETECT_TIMEOUT) {
                armSubsystem.setIntakeSpeed(0);
                System.out.println("timedout, switched to wait for note");
                state = IntakeCommand.State.WAIT_FOR_NOTE;
            }
            break;

        case RAISE_ARM:
            if (System.currentTimeMillis() - armUpStartTime >= ARM_UP_TIME) {
                armSubsystem.setAimPivotSpeed(0);
                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_NOTE_REVERSAL_REVERSE_SPEED);
                System.out.println("Switched to Reverse Note");
                state = IntakeCommand.State.REVERSE_NOTE;
            }
            break;

        case REVERSE_NOTE:
            if (!armSubsystem.isNoteDetected()) {
                noteForwardStartTime = System.currentTimeMillis();
                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_NOTE_REVERSAL_FORWARD_SPEED);
                System.out.println("Switched to forward Note");
                state = IntakeCommand.State.FORWARD_NOTE;
            }
            break;

        case FORWARD_NOTE:
            if ((System.currentTimeMillis() - noteForwardStartTime >= NOTE_FORWARD_TIME)) {
                armSubsystem.setIntakeSpeed(0);
                System.out.println("Switched to Note ready");
                state = IntakeCommand.State.NOTE_READY;
            }
            break;

        case NOTE_READY:
            if (!armSubsystem.isNoteDetected()) {
                System.out.println("Switched to wait for note from note ready");
                state = IntakeCommand.State.WAIT_FOR_NOTE;
            }
            break;

        case KILLED:
            if (armSubsystem.isNoteDetected()) {
                state = IntakeCommand.State.NOTE_READY;
            }
            else {
                state = IntakeCommand.State.WAIT_FOR_NOTE;
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
        state = IntakeCommand.State.KILLED;
    }

}
