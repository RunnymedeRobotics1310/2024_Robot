package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

// Shoot. That's it.
public class ShootCommand extends ArmBaseCommand {

    private enum State {
        START_INTAKE, STOP_INTAKE
    };

    private State state       = State.START_INTAKE;

    private long  startTimeMs = 0;

    public ShootCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {
        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            log(" No note detected in robot. ShootCommand cancelled");
            return;
        }

        logCommandStart();

    }

    @Override
    public void execute() {

        double intakeSpeed = 0;

        switch (state) {

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

            break;
        }
    }


}
