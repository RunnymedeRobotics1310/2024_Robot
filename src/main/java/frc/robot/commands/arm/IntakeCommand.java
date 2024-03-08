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
        WAIT_FOR_NOTE, NOTE_DETECTED, REVERSE_NOTE, FORWARD_NOTE, NOTE_READY, FINISHED, KILLED
    };

    private IntakeCommand.State state                = IntakeCommand.State.WAIT_FOR_NOTE;


    private long                noteForwardStartTime = 0;


    private long                intakeStartTime      = 0;

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
        state = State.WAIT_FOR_NOTE;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


        boolean atArmPosition = false;
        switch (state) {

        case WAIT_FOR_NOTE:
            // Notes should not be detected in this state but as a failsafe this will prevent the
            // state machine from getting stuck
            if (armSubsystem.isNoteDetected()) {

                System.out.println("safety code activated, switched to note detect");
                state = IntakeCommand.State.NOTE_DETECTED;
            }
            else {// This is the normal case
                armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_INTAKE_SPEED);
                intakeStartTime = System.currentTimeMillis();
                System.out.println("Switched to note detect");
                state = IntakeCommand.State.NOTE_DETECTED;
            }
            break;

        case NOTE_DETECTED:
            if (System.currentTimeMillis() - intakeStartTime >= INTAKE_SPINUP_WINDOW) {
                if (armSubsystem.getIntakeEncoderSpeed() <= 400) {
                    System.out.println(armSubsystem.getIntakeEncoderSpeed());
                    atArmPosition = this.driveToArmPosition(Constants.ArmConstants.OVER_BUMPER_POSITION, 5);
//                    if (atArmPosition) {
                    if (armSubsystem.isNoteDetected()) {

                        armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_NOTE_REVERSAL_REVERSE_SPEED);
                        System.out.println("Switched to Reverse Note");
                        state = IntakeCommand.State.REVERSE_NOTE;
                    }
                    else {
                        System.out.println("there should be a note in the thing");
                    }
//                    }
                }
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
                System.out.println("Switched to wait for note from note ready, this should not happen but it did");
                state = IntakeCommand.State.WAIT_FOR_NOTE;
            }
            else {
                state = State.FINISHED;
            }

            break;

        case FINISHED:

            break;

//            not currently being used
        case KILLED:
            if (armSubsystem.isNoteDetected()) {
                System.out.println("switched to note_ready from killed");
                state = IntakeCommand.State.NOTE_READY;
            }
            else {
                System.out.println("switched to wait for note from killed");
                state = IntakeCommand.State.WAIT_FOR_NOTE;
            }
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
            System.out.println("interupted");
//            Quentin said they had issues with interuptions last year, the killed state might work but idk, we arent using it
//            state = IntakeCommand.State.KILLED;
        }
        // run if not interupted
        else {
            System.out.println("command finished, moving to compact pose");
            CommandScheduler.getInstance().schedule(new CompactPoseCommand(armSubsystem));

        }

    }

}
