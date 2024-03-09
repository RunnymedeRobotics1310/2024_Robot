package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;

// Shoot. That's it.
public class ManualShootSimpleCommand extends ArmBaseCommand {

    private enum State {
        REVERSE_NOTE, WAIT_FOR_REVERSE, START_INTAKE, STOP_INTAKE, START_SHOOTER, STOP_SHOOTER
    };

    private State state            = State.REVERSE_NOTE;

    private long  startReverseTime = 0;

    public ManualShootSimpleCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {
        state = State.REVERSE_NOTE;

        logCommandStart();
    }

    @Override
    public void execute() {

        double intakeSpeed  = 0;
        double shooterSpeed = 0;

        switch (state) {

        case REVERSE_NOTE:
            startReverseTime = System.currentTimeMillis();
            armSubsystem.setShooterSpeed(-0.1);
            armSubsystem.setIntakeSpeed(-0.075);
            logStateTransition(State.WAIT_FOR_REVERSE.name(), "Reversed intake/shooter");
            state = State.WAIT_FOR_REVERSE;

        case WAIT_FOR_REVERSE:
            if (System.currentTimeMillis() - startReverseTime >= 300) {
                armSubsystem.setShooterSpeed(0);
                armSubsystem.setIntakeSpeed(0);
                logStateTransition(State.START_SHOOTER.name(), "Reverse done, move to start shooter");
                state = State.START_SHOOTER;
            }
            break;

        case START_SHOOTER:
            shooterSpeed = 0.75;
            armSubsystem.setShooterSpeed(shooterSpeed);
            if (((armSubsystem.getShooterEncoderSpeed())) >= 1310) {
                state = State.START_INTAKE;
            }

            break;

        case START_INTAKE:
            intakeSpeed = 1;
            armSubsystem.setIntakeSpeed(intakeSpeed);
            if (((armSubsystem.getIntakeEncoderSpeed())) >= 70) {
                state = State.STOP_INTAKE;
            }
            break;

        case STOP_INTAKE:
            intakeSpeed = 0;
            armSubsystem.setIntakeSpeed(intakeSpeed);
            state = State.STOP_SHOOTER;
            break;


        case STOP_SHOOTER:
            shooterSpeed = 0;
            armSubsystem.setShooterSpeed(shooterSpeed);
            break;
        }
    }

    public boolean isFinished() {
        if (state == State.STOP_SHOOTER) {
            armSubsystem.setShooterSpeed(0);
            return true;
        }
        return false;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);

        // stop motors if interupted
        if (interrupted) {
            armSubsystem.setIntakeSpeed(0);
            armSubsystem.setShooterSpeed(0);
        }
    }

}
