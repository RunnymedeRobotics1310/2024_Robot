package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.LoggingCommand;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends LoggingCommand {

    private final ArmSubsystem   armSubsystem;
    private final XboxController operatorController;
    private boolean shift;

    /**
     * Creates a new ExampleCommand.
     *
     * @param armSubsystem The subsystem used by this command.
     */
    public DefaultArmCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem) {

        this.operatorController = operatorInput.getOperatorController();
        this.armSubsystem       = armSubsystem;

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

        shift = operatorController.getRightBumper();
        if (!shift) {
            setLinkMotorSpeed(operatorController.getLeftY() * 0.5);
            setAimMotorSpeed(operatorController.getRightY() * 0.5);
        } else {
            setLinkMotorSpeed(0);
            setAimMotorSpeed(0);
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