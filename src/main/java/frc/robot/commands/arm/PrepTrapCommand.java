package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class PrepTrapCommand extends ArmBaseCommand {

    ClimbSubsystem climbSubsystem;

    public PrepTrapCommand(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem) {
        super(armSubsystem);
        this.climbSubsystem = climbSubsystem;
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {
        driveToArmPosition(Constants.ArmConstants.SHOOT_AMP_ARM_POSITION, 10, 10);
        climbSubsystem.setClimbSpeeds(1, 1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
