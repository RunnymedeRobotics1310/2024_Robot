package frc.robot.commands.arm;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class IntakeCommand extends ArmBaseCommand {

    private final JackmanVisionSubsystem jackmanVisionSubsystem;

    private static final long            NOTE_DETECT_TIMEOUT  = 1000;

    private static final long            NOTE_FORWARD_TIME    = 300;

    private static final long            INTAKE_SPINUP_WINDOW = 500;



    private enum State {
        WAIT_FOR_NOTE, WAIT_FOR_ARM, REVERSE_NOTE, FORWARD_NOTE, NOTE_READY, FINISHED
    };

    private IntakeCommand.State state                = IntakeCommand.State.WAIT_FOR_NOTE;


    private long                noteForwardStartTime = 0;


    private long                intakeStartTime      = 0;

    private boolean             noteAcquiredSuccess  = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param armSubsystem The subsystem used by this command.
     */
    public IntakeCommand(ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackmanVisionSubsystem) {
        super(armSubsystem);

        this.jackmanVisionSubsystem = jackmanVisionSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(jackmanVisionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();

        // Notes should not be detected in this state but as a failsafe this will prevent the
        // state machine from getting stuck
        if (armSubsystem.isNoteDetected()) {
            logStateTransition(State.NOTE_READY.name(), "Note already present/detected");
            state = State.NOTE_READY;
        }
        else {// This is the normal case
            armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_INTAKE_SPEED);
            intakeStartTime = System.currentTimeMillis();
            logStateTransition(State.WAIT_FOR_NOTE.name(), "Starting intake and waiting for note");
            state = IntakeCommand.State.WAIT_FOR_NOTE;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        switch (state) {

        case WAIT_FOR_NOTE:
            if (System.currentTimeMillis() - intakeStartTime >= INTAKE_SPINUP_WINDOW) {
                // If the intake is jammed, we need to move the arm to the over bumper position.
                // We shouldn't have a note detected at this point - this is safety code
                if (armSubsystem.getIntakeEncoderSpeed() <= 400 || armSubsystem.isNoteDetected()) {
                    logStateTransition(State.WAIT_FOR_ARM.name(), "Intake jammed (expected) or note detected");
                    state = State.WAIT_FOR_ARM;
                }
            }
            break;

        case WAIT_FOR_ARM:
            boolean atArmPosition = this.driveToArmPosition(Constants.ArmConstants.NOTE_INTAKE_CLEARANCE_POSITION, 5);
            if (atArmPosition) {
                if (armSubsystem.isNoteDetected()) {
                    armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_NOTE_REVERSAL_REVERSE_SPEED);
                    logStateTransition(State.REVERSE_NOTE.name(), "Arm In Position, ready for note positioning");
                    state = IntakeCommand.State.REVERSE_NOTE;
                }
                else {
                    logStateTransition(State.FINISHED.name(), "No note detected, SOMETHING WENT WRONG");
                    state = State.FINISHED;
                }
            }
            break;

        case REVERSE_NOTE:
            if (!armSubsystem.isNoteDetected()) {
                noteForwardStartTime = System.currentTimeMillis();
                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_NOTE_REVERSAL_FORWARD_SPEED);
                logStateTransition(State.FORWARD_NOTE.name(), "Note detection gone, note has reversed enough");
                state = IntakeCommand.State.FORWARD_NOTE;
            }
            break;

        case FORWARD_NOTE:
            if ((System.currentTimeMillis() - noteForwardStartTime >= NOTE_FORWARD_TIME)) {
                armSubsystem.setIntakeSpeed(0);
                logStateTransition(State.NOTE_READY.name(), "Note moved forward, we're ready");
                state = IntakeCommand.State.NOTE_READY;
            }
            break;

        case NOTE_READY:
            if (!armSubsystem.isNoteDetected()) {
                logStateTransition(State.WAIT_FOR_NOTE.name(), "Note lost, which shouldn't happen");
                state = IntakeCommand.State.WAIT_FOR_NOTE;
            }
            else {
                noteAcquiredSuccess = true;
                logStateTransition(State.FINISHED.name(), "Command complete");
                state = State.FINISHED;
            }
            break;

        case FINISHED:
            break;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The default arm command never ends, but can be interrupted by other commands.
        return state == State.FINISHED;

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // run if interupted
        if (interrupted) {
            logCommandEnd(interrupted);
            armSubsystem.setIntakeSpeed(0);
        }
        // run if not interupted
        else {
            if (noteAcquiredSuccess) {
                logCommandEnd(interrupted, "command finished, moving to compact pose");
                CommandScheduler.getInstance().schedule(new CompactPoseCommand(armSubsystem));
            }
            else {
                logCommandEnd(interrupted, "command finished, no note acquired");
            }
        }

    }

}
