package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class IntakeSimpleCommand extends ArmBaseCommand {

    private enum State {
        WAIT_FOR_NOTE, WAIT_FOR_ARM, NOTE_READY, FINISHED
    };

    private IntakeSimpleCommand.State state                = IntakeSimpleCommand.State.WAIT_FOR_NOTE;

    private long                      intakeStartTime      = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param armSubsystem The subsystem used by this command.
     */
    public IntakeSimpleCommand(ArmSubsystem armSubsystem) {
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
            armSubsystem.setIntakeSpeed(ArmConstants.INTAKE_INTAKE_SPEED);
            intakeStartTime = System.currentTimeMillis();
            logStateTransition(State.WAIT_FOR_NOTE.name(), "Starting intake and waiting for note");
            state = IntakeSimpleCommand.State.WAIT_FOR_NOTE;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        switch (state) {

        case WAIT_FOR_NOTE:
            if (System.currentTimeMillis() - intakeStartTime >= ArmConstants.INTAKE_SPINUP_WINDOW) {
                // If the intake is jammed, we need to move the arm to the over bumper position.
                // We shouldn't have a note detected at this point - this is safety code
                if (armSubsystem.getIntakeEncoderSpeed() <= 400 || armSubsystem.isNoteDetected()) {
                    logStateTransition(State.WAIT_FOR_ARM.name(), "Intake jammed (expected) or note detected");
                    state = State.WAIT_FOR_ARM;
                }
            }
            break;

        case WAIT_FOR_ARM:
            boolean atArmPosition = this.driveThroughArmPosition(ArmConstants.NOTE_INTAKE_CLEARANCE_POSITION,
                ArmConstants.DEFAULT_LINK_TOLERANCE_DEG, ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);
            if (atArmPosition) {
                armSubsystem.setIntakeSpeed(0);
                logStateTransition(State.FINISHED.name(), "No note detected, SOMETHING WENT WRONG");
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
            logCommandEnd(interrupted, "command finished, moving to compact pose");
            CommandScheduler.getInstance().schedule(new CompactPoseCommand(armSubsystem));
        }

    }

}
