package frc.robot.commands.arm;

import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.SOURCE_INTAKE_POSE;

public class AimSourceCommand extends ArmBaseCommand {

    // todo fixme: use constants for all

    public AimSourceCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void execute() {
        driveToArmPosition(SOURCE_INTAKE_POSE, 5, 50);
    }

    @Override
    public boolean isFinished() {
        return driveToArmPosition(SOURCE_INTAKE_POSE, 5, 50);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            logCommandEnd(interrupted);
            armSubsystem.stop();
        }
    }
}