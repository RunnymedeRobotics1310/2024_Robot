package frc.robot.commands.arm;

import static frc.robot.Constants.LightingConstants.SIGNAL;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Shooting;

// Shoot. That's it.
public class ShootTrapFromFloorCommand extends ArmBaseCommand {

    private enum State {
        REVERSE_NOTE, START_SHOOTER, START_FEEDER, FINISHED
    };

    private State             state               = State.REVERSE_NOTE;
    private LightingSubsystem lighting;

    private double            startIntakePosition = 0;

    private double            topShooterSpeed     = .3;
    private double            bottomShooterSpeed  = .4;

    public ShootTrapFromFloorCommand(ArmSubsystem armSubsystem, LightingSubsystem lighting, double topShooterSpeed, double bottomShooterSpeed) {

        super(armSubsystem);
        this.lighting = lighting;
        this.topShooterSpeed = topShooterSpeed;
        this.bottomShooterSpeed = bottomShooterSpeed;
    }

    @Override
    public void initialize() {

        state               = State.REVERSE_NOTE;

        startIntakePosition = armSubsystem.getIntakePosition();

        logCommandStart("Intake Position " + startIntakePosition);
        lighting.addPattern(SIGNAL, Shooting.getInstance());
    }

    @Override
    public void execute() {

        double intakeSpeed  = 0;
        double shooterSpeed = 0;

        switch (state) {

        case REVERSE_NOTE:

            armSubsystem.setShooterSpeed(-0.1);
            armSubsystem.setIntakeSpeed(-0.3);

            // Reverse the note for a number of rotations
            if (Math.abs(armSubsystem.getIntakePosition() - startIntakePosition) > 2) {
                logStateTransition("Reverse -> Start Shooter", "Shooter Reversed");
                state = State.START_SHOOTER;
            }

            break;

        case START_SHOOTER:

            armSubsystem.setIntakeSpeed(0);
            armSubsystem.setShooterSpeed(topShooterSpeed, bottomShooterSpeed);

            // Wait for the shooter to get up to speed
            if (isStateTimeoutExceeded(.5)) {
                logStateTransition("Start Shooter -> Shoot", "Shooter up to speed " + armSubsystem.getBottomShooterEncoderSpeed());
                state = State.START_FEEDER;
            }

            break;

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

        if (state == State.FINISHED) {
            return true;
        }
        return false;
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
