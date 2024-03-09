package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

// Shoot. That's it.
public class ManualShootCommand extends ArmBaseCommand {

    private enum State {
        START_INTAKE, STOP_INTAKE, START_SHOOTER, STOP_SHOOTER
    };

    private State state       = State.START_SHOOTER;

    private long  startTimeMs = 0;

    public ManualShootCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {
        startTimeMs = System.currentTimeMillis();
        state = State.START_SHOOTER;
        // If there is no note detected, then why are we aiming?
//        if (!armSubsystem.isNoteDetected()) {
//            System.out.println(" No note detected in robot. ShootCommand cancelled");
//            return;
//        }

        logCommandStart();

    }

    @Override
    public void execute() {

        double intakeSpeed  = 0;
        double shooterSpeed = 0;

        switch (state) {

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

        case START_SHOOTER:

            shooterSpeed = 0.75;
            armSubsystem.setShooterSpeed(shooterSpeed);
            if (((armSubsystem.getShooterEncoderSpeed())) >= 1310) {
                state = State.START_INTAKE;
            }

            break;

        case STOP_SHOOTER:

            shooterSpeed = 0;
            armSubsystem.setShooterSpeed(shooterSpeed);

            break;
        }
    }

    public boolean isFinished() {
        if ( state == State.STOP_SHOOTER) {
            armSubsystem.setShooterSpeed(0);
            return true;
        }
        return false;
    }

}
