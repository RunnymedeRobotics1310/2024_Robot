package frc.robot.commands.arm;

import static frc.robot.Constants.LightingConstants.SIGNAL;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Shooting;

// Shoot. That's it.
public class ShootPrepFireCommand extends ArmBaseCommand {

    private enum State {
        REVERSE_NOTE, START_SHOOTER, WAIT_FOR_IT, START_FEEDER, FINISHED
    };

    private State             state               = State.REVERSE_NOTE;
    private LightingSubsystem lighting;
    private OperatorInput operatorInput;

    private double            startIntakePosition = 0;

    public ShootPrepFireCommand(ArmSubsystem armSubsystem, LightingSubsystem lighting, OperatorInput operatorInput) {

        super(armSubsystem);
        this.lighting = lighting;
        this.operatorInput = operatorInput;
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
            if (Math.abs(armSubsystem.getIntakePosition() - startIntakePosition) > 1.5) {
                setStateAndLog(State.START_SHOOTER, "Shooter Reversed");
            }

            break;

        case START_SHOOTER:

            armSubsystem.setIntakeSpeed(0);
            armSubsystem.setShooterSpeed(.75);

            setStateAndLog(State.WAIT_FOR_IT, "Shooter started, wait for it to spin up and button release");
            break;

        case WAIT_FOR_IT:
            // Wait for the shooter to get up to speed, and button to be released
            double elapsedTime = getStateElapsedTime();

            if (elapsedTime >= .5 && operatorInput.getRawOperatorController().getAButtonReleased()) {
                setStateAndLog(State.START_FEEDER, "ShooterSpeed[" + armSubsystem.getBottomShooterEncoderSpeed() + "], Elapsed[" + elapsedTime + "]");
            }
            break;

        case START_FEEDER:

            armSubsystem.setIntakeSpeed(1);

            if (isStateTimeoutExceeded(.5)) {
                setStateAndLog(State.FINISHED, "Shot fired");
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

    /**
     * Set the state and log the transition
     *
     * @param newState State to transition to
     * @param reason Reason for the transition for logging
     */
    private void setStateAndLog(State newState, String reason) {
        logStateTransition(newState.name(), reason);
        state = newState;
    }


}
