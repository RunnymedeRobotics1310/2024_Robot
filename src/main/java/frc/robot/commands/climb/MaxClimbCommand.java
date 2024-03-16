package frc.robot.commands.climb;

import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class MaxClimbCommand extends BaseClimbCommand {

    private enum State {
        RAISE_CLIMB, WAIT_FOR_CLIMB, FINISHED
    };

    private MaxClimbCommand.State state = State.RAISE_CLIMB;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public MaxClimbCommand(ClimbSubsystem climbSubsystem, SwerveSubsystem drive) {
        super(climbSubsystem, drive);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        switch (state) {
        case RAISE_CLIMB:
            climbSubsystem.setClimbSpeeds(ClimbConstants.RAISE_CLIMBERS_SPEED, ClimbConstants.RAISE_CLIMBERS_SPEED);
            logStateTransition(State.WAIT_FOR_CLIMB.name(), "Waiting for arms and all to be raised");
            state = State.WAIT_FOR_CLIMB;
            break;

        case WAIT_FOR_CLIMB:
            if (climbSubsystem.isRightClimbAtMax() && climbSubsystem.isLeftClimbAtMax()) {
                logStateTransition(State.FINISHED.name(), "In position");
                state = State.FINISHED;
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The default drive command never ends, but can be interrupted by other commands.
        return state == State.FINISHED;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);

        if (interrupted) {
            climbSubsystem.setClimbSpeeds(0, 0);
        }
    }
}
