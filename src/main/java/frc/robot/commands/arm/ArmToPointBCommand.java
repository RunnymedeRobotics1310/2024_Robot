package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Raise the arm up safely above bumper from intake position to permit safe arm movement to other locations
public class ArmToPointBCommand extends ArmBaseCommand {

    public ArmToPointBCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void execute() {
        driveToArmPosition(ArmConstants.OVER_BUMPER_POSITION, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
            ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);
    }

    @Override
    public boolean isFinished() {
        return isAtArmPosition(ArmConstants.OVER_BUMPER_POSITION, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
        logCommandEnd(interrupted);
    }

}
