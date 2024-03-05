package frc.robot.commands.climb;

import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.operator.GameController;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ClimbSubsystem;

public class DefaultClimbCommand extends LoggingCommand {

    private final ClimbSubsystem climbSubsystem;
    private final GameController operatorController;
    private boolean shift;
    double leftClimbSpeed = 0;
    double rightClimbSpeed = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public DefaultClimbCommand(OperatorInput operatorInput, ClimbSubsystem climbSubsystem) {

        this.operatorController = operatorInput.getOperatorController();
        this.climbSubsystem     = climbSubsystem;

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

        shift = operatorController.getRightBumper();

        // Pull the robot up (arms down, -ve motor speed) using operator triggers
        // NOTE: Left trigger is already negative.
        if (shift) {
            leftClimbSpeed = operatorController.getLeftY() * ClimbConstants.MAX_ROBOT_LIFT_SPEED;
            rightClimbSpeed = operatorController.getRightY() * ClimbConstants.MAX_ROBOT_LIFT_SPEED;
        } else {
            leftClimbSpeed = 0;
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