package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class SimpleAmpPositionCommand extends ArmBaseCommand {
    public SimpleAmpPositionCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void execute() {
        driveToArmPosition(Constants.ArmConstants.SHOOT_AMP_ARM_POSITION, Constants.ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
            Constants.ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
