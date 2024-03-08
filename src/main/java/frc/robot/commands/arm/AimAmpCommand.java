package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Move arm to amp shoot pose
// Set shooter speed
public class AimAmpCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_AMP, MOVE_TO_UNLOCK, MOVE_TO_OVER_BUMPER, SET_SHOOTER_SPEED
    };

    private State state = State.MOVE_TO_AMP;

    public AimAmpCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {

        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            System.out.println(" No note detected in robot. AimAmpCommand cancelled");
            return;
        }

        logCommandStart();

        if (armSubsystem.getAimAngle() < ArmConstants.UNLOCK_POSITION.aimAngle) {
            state = State.MOVE_TO_UNLOCK;
        }
        else if (armSubsystem.getLinkAngle() < ArmConstants.OVER_BUMPER_POSITION.linkAngle) {
            state = State.MOVE_TO_OVER_BUMPER;
        }
        else {
            state = State.MOVE_TO_AMP;
        }
    }

    @Override
    public void execute() {

        boolean atArmAngle = false;

        switch (state) {

        case MOVE_TO_AMP:
            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.SHOOT_AMP_ARM_POSITION, 5);
            if (atArmAngle) {
                logStateTransition("Start Shooter", "Arm at Shoot Amp position");
                state = State.SET_SHOOTER_SPEED;
            }


        case MOVE_TO_OVER_BUMPER:

            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.OVER_BUMPER_POSITION, 5);

            // If past the bumper danger, move to the amp position.

            // If the aim is higher than the over-the-bumper angle, then it is safe to start
            // raising the link to the amp position.
            if (atArmAngle) {
                logStateTransition("Move to Amp", "Arm at over bumper position");
                state = State.MOVE_TO_AMP;
            }

            break;

        case MOVE_TO_UNLOCK:

            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.UNLOCK_POSITION, 5);

            // If past the bumper danger, move to the compact position.
            if (atArmAngle) {
                logStateTransition("Move to over bumper", "Arm at unlock position");
                state = State.MOVE_TO_OVER_BUMPER;
            }

            break;

        case SET_SHOOTER_SPEED:
            armSubsystem.setShooterSpeed(ArmConstants.SHOOTER_AMP_SPEED);

            break;
        }
    }
}