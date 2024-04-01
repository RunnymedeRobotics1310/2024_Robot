package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Raise the arm up safely above bumper from intake position to permit safe arm movement to other locations
public class RaiseFromIntakeCommand extends ArmBaseCommand {

    public RaiseFromIntakeCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.setLinkPivotSpeed(.5);
    }

    @Override
    public boolean isFinished() {
        return (armSubsystem.getLinkAngle() > ArmConstants.INTAKE_ARM_POSITION.linkAngle + 4);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
        logCommandEnd(interrupted);
    }

}
