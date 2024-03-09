package frc.robot.commands.climb;

import static frc.robot.commands.operator.OperatorInput.Axis.Y;
import static frc.robot.commands.operator.OperatorInput.Stick.LEFT;
import static frc.robot.commands.operator.OperatorInput.Stick.RIGHT;

import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ClimbSubsystem;

public class DefaultClimbCommand extends LoggingCommand {

    private final ClimbSubsystem climbSubsystem;
    private final OperatorInput  operatorInput;
    double                       leftClimbSpeed  = 0;
    double                       rightClimbSpeed = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public DefaultClimbCommand(OperatorInput operatorInput, ClimbSubsystem climbSubsystem) {

        this.operatorInput  = operatorInput;
        this.climbSubsystem = climbSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


        // Pull the robot up (arms down, -ve motor speed) using operator triggers
        // NOTE: Left trigger is already negative.
        if (operatorInput.isShift()) {
            leftClimbSpeed  = operatorInput.getOperatorControllerAxis(LEFT, Y) * ClimbConstants.MAX_ROBOT_LIFT_SPEED;
            rightClimbSpeed = operatorInput.getOperatorControllerAxis(RIGHT, Y) * ClimbConstants.MAX_ROBOT_LIFT_SPEED;
        }
        else {
            leftClimbSpeed  = 0;
            rightClimbSpeed = 0;
        }
        // Bumpers lift the arms up in order to catch the chain

        climbSubsystem.setClimbSpeeds(leftClimbSpeed, rightClimbSpeed);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The default drive command never ends, but can be interrupted by other commands.
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}