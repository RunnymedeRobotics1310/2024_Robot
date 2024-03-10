package frc.robot.commands.arm;

import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.SOURCE_INTAKE_POSE;

public class LinkToSourceCommand extends ArmBaseCommand {

    // todo fixme: use constants for all

    public LinkToSourceCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }


    @Override
    public void initialize() {

        logCommandStart();
    }


    @Override
    public void execute() {
        driveToArmPosition(SOURCE_INTAKE_POSE, 5, 50);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return driveToArmPosition(SOURCE_INTAKE_POSE, 5, 50);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // run if interupted
        if (interrupted) {
            logCommandEnd(interrupted);
            armSubsystem.stop();
        }
    }
}