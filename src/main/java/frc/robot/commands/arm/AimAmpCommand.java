package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Move arm to amp shoot pose
// Set shooter speed
public class AimAmpCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_AMP, MOVE_TO_OVER_BUMPER, SET_SHOOTER_SPEED
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

        if (armSubsystem.getAimAngle() > ArmConstants.OVER_BUMPER_POSITION.aimAngle) {
            state = State.MOVE_TO_OVER_BUMPER;
        }
        else {
            state = State.MOVE_TO_AMP;
        }
    }

    @Override
    public void execute() {

        // Get the current angles
        double currentLinkAngle = armSubsystem.getLinkAngle();
        double currentAimAngle  = armSubsystem.getAimAngle();

        double linkAngleError   = 0;
        double aimAngleError    = 0;

        double linkSpeed        = 0;
        double aimSpeed         = 0;

        switch (state) {

        case MOVE_TO_AMP:

            // Calculate the errors

            linkAngleError = ArmConstants.SHOOT_AMP_ARM_POSITION.linkAngle - currentLinkAngle;
            aimAngleError = ArmConstants.SHOOT_AMP_ARM_POSITION.aimAngle - currentAimAngle;

            // Move the motor with the larger error at the appropriate speed (Fast or Slow)
            // depending on the error
            if (Math.abs(aimAngleError) >= Math.abs(linkAngleError)) {

                aimSpeed = ArmConstants.FAST_AIM_SPEED;

                if (Math.abs(aimAngleError) < ArmConstants.SLOW_ARM_ZONE_DEG) {
                    aimSpeed = ArmConstants.SLOW_AIM_SPEED;
                }

                if (Math.abs(aimAngleError) < ArmConstants.AT_TARGET_DEG) {
                    aimSpeed = 0;
                }

                // Set the link speed relative to the aim speed.
                if (Math.abs(linkAngleError) > ArmConstants.AT_TARGET_DEG) {
                    linkSpeed = Math.abs((linkAngleError / aimAngleError)) * aimSpeed;
                }
            }
            else {

                linkSpeed = ArmConstants.FAST_LINK_SPEED;

                if (Math.abs(linkAngleError) < ArmConstants.SLOW_ARM_ZONE_DEG) {
                    linkSpeed = ArmConstants.SLOW_LINK_SPEED;
                }

                if (Math.abs(linkAngleError) < ArmConstants.AT_TARGET_DEG) {
                    linkSpeed = 0;
                }

                // Set the aim speed relative to the link speed.
                if (Math.abs(aimAngleError) > ArmConstants.AT_TARGET_DEG) {
                    aimSpeed = Math.abs((aimAngleError / linkAngleError)) * linkSpeed;
                }
            }

            if (aimAngleError < 0) {
                aimSpeed *= -1.0;
            }

            if (linkAngleError < 0) {
                linkSpeed *= -1.0;
            }

            armSubsystem.setLinkPivotSpeed(linkSpeed);
            armSubsystem.setAimPivotSpeed(aimSpeed);

            break;

        case MOVE_TO_OVER_BUMPER:

            // Calculate the errors

            linkAngleError = ArmConstants.OVER_BUMPER_POSITION.linkAngle - currentLinkAngle;
            aimAngleError = ArmConstants.OVER_BUMPER_POSITION.aimAngle - currentAimAngle;

            // Move the motor with the larger error at the appropriate speed (Fast or Slow)
            // depending on the error
            if (Math.abs(aimAngleError) >= Math.abs(linkAngleError)) {

                aimSpeed = ArmConstants.FAST_AIM_SPEED;

                // Set the link speed relative to the aim speed.
                if (Math.abs(linkAngleError) > ArmConstants.AT_TARGET_DEG) {
                    linkSpeed = Math.abs((linkAngleError / aimAngleError)) * aimSpeed;
                }
            }
            else {

                linkSpeed = ArmConstants.FAST_LINK_SPEED;

                // Set the aim speed relative to the link speed.
                if (Math.abs(aimAngleError) > ArmConstants.AT_TARGET_DEG) {
                    aimSpeed = Math.abs((aimAngleError / linkAngleError)) * linkSpeed;
                }
            }

            if (aimAngleError < 0) {
                aimSpeed *= -1.0;
            }

            if (linkAngleError < 0) {
                linkSpeed *= -1.0;
            }

            armSubsystem.setLinkPivotSpeed(linkSpeed);
            armSubsystem.setAimPivotSpeed(aimSpeed);

            // If past the bumper danger, move to the amp position.

            // If the aim is higher than the over-the-bumper angle, then it is safe to start
            // raising the link to the amp position.
            if (aimAngleError < 0) {
                logStateTransition("Move to amp", "Aim over bumper position");
                state = State.MOVE_TO_AMP;
            }

            break;

        case SET_SHOOTER_SPEED:
            armSubsystem.setShooterSpeed(ArmConstants.SHOOTER_AMP_SPEED);

            break;
        }
    }
}