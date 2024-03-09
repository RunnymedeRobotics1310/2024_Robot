package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class IntakeEjectCommand extends ArmBaseCommand {
    public IntakeEjectCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);

    }

    private enum State {
        EJECT
    };

    private State state = State.EJECT;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        switch (state) {
        case EJECT:
            armSubsystem.setIntakeSpeed(Constants.ArmConstants.INTAKE_NOTE_REVERSAL_REVERSE_SPEED);

            break;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // run if interupted
        if (interrupted) {
            logCommandEnd(interrupted);
            armSubsystem.setIntakeSpeed(0);
        }
        // run if not interupted


    }

}


