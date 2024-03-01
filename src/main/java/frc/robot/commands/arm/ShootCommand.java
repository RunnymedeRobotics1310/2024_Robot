package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

// Shoot. That's it.
public class ShootCommand extends ArmBaseCommand {

    private enum State {
        START_INTAKE, STOP_INTAKE, START_SHOOTER, STOP_SHOOTER
    };

    private State state       = State.START_SHOOTER;

    private long  startTimeMs = 0;

    public ShootCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {
        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            System.out.println(" No note detected in robot. ShootCommand cancelled");
            return;
        }

        logCommandStart();

    }

    @Override
    public void execute() {

        double shooterSpeed = 0;
        double intakeSpeed  = 0;

        switch (state) {

        case START_SHOOTER:

            shooterSpeed = 1;
            armSubsystem.setShooterSpeed(shooterSpeed);
            startTimeMs = System.currentTimeMillis();
            if (((System.currentTimeMillis() - startTimeMs) / 1000.0d) > 0.25) {
                state = State.START_INTAKE;
            }

            break;

        case STOP_SHOOTER:

            shooterSpeed = 0;
            armSubsystem.setShooterSpeed(shooterSpeed);

            break;

        case START_INTAKE:

            intakeSpeed = 1;
            armSubsystem.setIntakeSpeed(intakeSpeed);
            startTimeMs = System.currentTimeMillis();
            if (((System.currentTimeMillis() - startTimeMs) / 1000.0d) > 1) {
                state = State.STOP_INTAKE;
            }

            break;

        case STOP_INTAKE:

            intakeSpeed = 0;
            armSubsystem.setIntakeSpeed(intakeSpeed);

            state = State.STOP_SHOOTER;

            break;
        }
    }


}
