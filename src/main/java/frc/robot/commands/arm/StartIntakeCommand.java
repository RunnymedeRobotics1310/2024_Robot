package frc.robot.commands.arm;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

// Intake until there is a note detected.
public class StartIntakeCommand extends LoggingCommand {

    private final ArmSubsystem armSubsystem;

    private long               noteDetectTime = Long.MAX_VALUE;

    private boolean            noteDetected   = false;

    public StartIntakeCommand(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        noteDetected = false;

        logCommandStart();
    }

    @Override
    public void execute() {

        armSubsystem.setShooterSpeed(-.2);
        armSubsystem.setIntakeSpeed(0);

        if (armSubsystem.isNoteDetected() && !noteDetected) {
            noteDetectTime = System.currentTimeMillis();
            noteDetected   = true;
            armSubsystem.setIntakeSpeed(-.15);
            armSubsystem.setShooterSpeed(0);
        }

    }

    @Override
    public boolean isFinished() {

        if (System.currentTimeMillis() - noteDetectTime > 400) {
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
