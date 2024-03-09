package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class IntakeBackwardsCommand extends ArmBaseCommand {

    private static final long            NOTE_DETECT_TIMEOUT = 1000;

    private static final long            NOTE_FORWARD_TIME   = 300;



    private enum State {
        WAIT_FOR_NOTE, REVERSE_NOTE, NOTE_READY, FINISHED
    };

    private IntakeBackwardsCommand.State state                = IntakeBackwardsCommand.State.WAIT_FOR_NOTE;


    private long                noteForwardStartTime = 0;


    private long                gotNoteTime      = 0;

    private boolean             noteAcquiredSuccess  = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param armSubsystem The subsystem used by this command.
     */
    public IntakeBackwardsCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
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
            armSubsystem.setShooterSpeed(ArmConstants.INTAKE_NOTE_REVERSAL_REVERSE_SPEED);
            logStateTransition(State.WAIT_FOR_NOTE.name(), "Starting intake and waiting for note");
            state = IntakeBackwardsCommand.State.WAIT_FOR_NOTE;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        switch (state) {

        case WAIT_FOR_NOTE:
            if (armSubsystem.isNoteDetected()) {
                gotNoteTime = System.currentTimeMillis();
                armSubsystem.setIntakeSpeed(ArmConstants.INTAKE_NOTE_REVERSAL_REVERSE_SPEED);
                armSubsystem.setShooterSpeed(-0.1);
                logStateTransition(State.REVERSE_NOTE.name(), "Intake jammed (expected) or note detected");
                    state = State.REVERSE_NOTE;
            }
            break;

        case REVERSE_NOTE:
            if (System.currentTimeMillis()-gotNoteTime >= 300) {
                armSubsystem.setIntakeSpeed(0);
                armSubsystem.setShooterSpeed(0);
                logStateTransition(State.FINISHED.name(), "Note moved forward, we're ready");
                state = IntakeBackwardsCommand.State.FINISHED;
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
            armSubsystem.setShooterSpeed(0);
        }

    }

}
