package frc.robot.commands.arm;

import static frc.robot.Constants.LightingConstants.SIGNAL;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Shooting;

// Shoot. That's it.
public class ShootFireCommand extends ArmBaseCommand {

    private enum State {
        START_FEEDER, FINISHED
    };

    private State             state               = State.START_FEEDER;
    private LightingSubsystem lighting;

    public ShootFireCommand(ArmSubsystem armSubsystem, LightingSubsystem lighting) {

        super(armSubsystem);
        this.lighting = lighting;
    }

    @Override
    public void initialize() {

        state               = State.START_FEEDER;

        logCommandStart("Start Feeder to FIRE!");
        lighting.addPattern(SIGNAL, Shooting.getInstance());

    }

    @Override
    public void execute() {

        double intakeSpeed  = 0;
        double shooterSpeed = 0;

        switch (state) {

        case START_FEEDER:

            armSubsystem.setIntakeSpeed(1);

            if (isStateTimeoutExceeded(.5)) {
                logStateTransition("Shoot -> Finished", "Shot fired");
                state = State.FINISHED;
            }
            break;

        default:
            break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        armSubsystem.setIntakeSpeed(0);
        armSubsystem.setShooterSpeed(0);
        lighting.removePattern(Shooting.class);
        logCommandEnd(interrupted);

        if (!interrupted) {
            if (DriverStation.isTeleop()) {
                CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
            }

        }
    }

}
