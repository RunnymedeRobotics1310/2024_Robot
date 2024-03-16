package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class EjectNoteCommand extends ArmBaseCommand {

    public EjectNoteCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        armSubsystem.setIntakeSpeed(ArmConstants.INTAKE_EJECT_INTAKE_SPEED);
        armSubsystem.setShooterSpeed(ArmConstants.INTAKE_EJECT_SHOOTER_SPEED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setIntakeSpeed(0);
        armSubsystem.setShooterSpeed(0);
        logCommandEnd(interrupted);
    }
}


