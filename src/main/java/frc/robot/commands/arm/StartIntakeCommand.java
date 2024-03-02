package frc.robot.commands.arm;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

// Intake until there is a note detected.
public class StartIntakeCommand extends LoggingCommand {

    private final ArmSubsystem armSubsystem;

    private long               noteDetectTime = Long.MAX_VALUE;

    public StartIntakeCommand(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        logCommandStart();
    }

    @Override
    public void execute() {

        armSubsystem.setShooterSpeed(-.2);
        armSubsystem.setIntakeSpeed(0);

        if (armSubsystem.isNoteDetected()) {
            noteDetectTime = System.currentTimeMillis();
            armSubsystem.setIntakeSpeed(-.15);
            armSubsystem.setShooterSpeed(0);
        }

        if (isTimeoutExceeded(1.5)) {
            armSubsystem.setIntakeSpeed(.3);
        }
    }

    @Override
    public boolean isFinished() {

        if (System.currentTimeMillis() - noteDetectTime > 250) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);

        armSubsystem.stop();
    }


}
