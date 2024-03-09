package frc.robot.commands.arm;

import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;

public class LinkToSourceCommand extends ArmBaseCommand {

    // todo fixme: use constants for all

    public LinkToSourceCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }


    @Override
    public void initialize() {

        logCommandStart();
    }

    ArmPosition sourceIntake = new ArmPosition(200, 35);

    @Override
    public void execute() {
        driveToArmPosition(sourceIntake, 5, 50);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return driveToArmPosition(sourceIntake, 5, 50);
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