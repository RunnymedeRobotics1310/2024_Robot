package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

// Move arm to speaker shoot pose
// Set shooter speed (distance based)
public class AimSpeakerCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_SPEAKER, MOVE_TO_UNLOCK, MOVE_TO_OVER_BUMPER, SET_SHOOTER_SPEED
    };

    private State state = State.MOVE_TO_SPEAKER;

    public AimSpeakerCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {

        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            System.out.println("No note detected in robot. AimSpeakerCommand cancelled");
            return;
        }

        logCommandStart();

        if (armSubsystem.getAimAngle() > ArmConstants.OVER_BUMPER_POSITION.aimAngle) {
            state = State.MOVE_TO_OVER_BUMPER;
        }
        else {
            state = State.MOVE_TO_UNLOCK;
        }
    }

    @Override
    public void execute() {

        boolean atArmAngle = false;


        switch (state) {

        case MOVE_TO_SPEAKER:
            // Move to the requested angle with a tolerance of 5 deg
            if (HughVisionSubsystem.getDynamicSpeakerShooterAngle == Double.MIN_VALUE) {
                atArmAngle = this.driveToArmPosition(ArmConstants.SHOOT_SPEAKER_ARM_POSITION, 5);
            }
            else {
                atArmAngle = this.driveToArmPosition(ArmConstants.SHOOT_SPEAKER_ARM_POSITION.linkAngle,
                    HughVisionSubsystem.getDynamicSpeakerShooterAngle - ArmConstants.SHOOTER_AIM_DIFFERENCE, 5);
            }

            if (atArmAngle) {
                logStateTransition("Start Shooter", "Arm at Shooter Position");
                state = State.MOVE_TO_SPEAKER;
            }
            break;

        case MOVE_TO_OVER_BUMPER:

            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.OVER_BUMPER_POSITION, 5);

            // If past the bumper danger, move to the speaker position.

            // If the aim is higher than the over-the-bumper angle, then it is safe to start
            // raising the link to the speaker position.
            if (atArmAngle) {
                logStateTransition("Move to Speaker", "Arm over bumper");
                state = State.MOVE_TO_SPEAKER;
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

        case SET_SHOOTER_SPEED:
            armSubsystem.setShooterSpeed(ArmConstants.SHOOTER_SPEAKER_SPEED);

            break;

        }
    }
}