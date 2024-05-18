package frc.robot.commands.arm;

import static frc.robot.commands.operator.OperatorInput.Axis.Y;
import static frc.robot.commands.operator.OperatorInput.Stick.LEFT;
import static frc.robot.commands.operator.OperatorInput.Stick.RIGHT;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends ArmBaseCommand {

    private final ArmSubsystem  armSubsystem;
    private final OperatorInput operatorInput;

    private double              linkAngle = -1;
    private double              aimAngle  = -1;

    /**
     * Creates a new ExampleCommand.
     *
     * @param armSubsystem The subsystem used by this command.
     */
    public DefaultArmCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem) {
        super(armSubsystem);

        this.operatorInput = operatorInput;
        this.armSubsystem  = armSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
        aimAngle  = armSubsystem.getAimAngle();
        linkAngle = armSubsystem.getLinkAngle();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double aimStick  = operatorInput.getOperatorControllerAxis(RIGHT, Y);
        double linkStick = operatorInput.getOperatorControllerAxis(LEFT, Y);

        // Hold the arm position by default
        driveToArmPosition(linkAngle, aimAngle, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
            ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

        // // Adjust the target angle based on the operator inputs.
        // aimAngle += operatorInput.getAimAdjust();
        // linkAngle += operatorInput.getLinkAdjust();

        if (!operatorInput.isShift() &&
            (Math.abs(linkStick) > 0 || Math.abs(aimStick) > 0)) {
            aimAngle  = armSubsystem.getAimAngle();
            linkAngle = armSubsystem.getLinkAngle();
            setLinkMotorSpeed(linkStick * 0.3);
            setAimMotorSpeed(aimStick * 0.5);

        }
        else {
            // driveToArmPosition(linkAngle, aimAngle, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
            // ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);
            setAimMotorSpeed(0);
            setLinkMotorSpeed(0);
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