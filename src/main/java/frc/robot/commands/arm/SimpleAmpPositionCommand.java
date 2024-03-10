package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class SimpleAmpPositionCommand extends ArmBaseCommand {
    public SimpleAmpPositionCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void execute() {
        if (driveToArmPosition(Constants.ArmConstants.SHOOT_AMP_ARM_POSITION, Constants.ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
            Constants.ArmConstants.DEFAULT_AIM_TOLERANCE_DEG)) {
            armSubsystem.setAimPivotSpeed(0.02);
        }
    }

    @Override
    public boolean isFinished() {
        // return driveToArmPosition(Constants.ArmConstants.SHOOT_AMP_ARM_POSITION,
        // Constants.ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
        // Constants.ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

        // button set to whileTrue
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        armSubsystem.stop();
    }
}
