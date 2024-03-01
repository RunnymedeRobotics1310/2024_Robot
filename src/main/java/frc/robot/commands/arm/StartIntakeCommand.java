package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Start Intake
// Move Aim/Arm
public class StartIntakeCommand extends ArmBaseCommand {

    private enum State {
        MOVE_AIM_ABOVE_90, MOVE_TO_OVER_BUMPER, MOVE_TO_INTAKE
    };

    private State state = State.MOVE_AIM_ABOVE_90;

    public StartIntakeCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void initialize() {

        // If there is a note inside the robot, then do not start this command
        if (armSubsystem.isNoteDetected()) {
            System.out.println("Note detected in robot. StartIntakeCommand cancelled");
            return;
        }

        logCommandStart();

        if (armSubsystem.getAimAngle() < 90) {
            state = State.MOVE_AIM_ABOVE_90;
        }
        else if (armSubsystem.getAimAngle() < ArmConstants.OVER_BUMPER_POSITION.aimAngle) {
            state = State.MOVE_TO_OVER_BUMPER;
        }
        else {
            state = State.MOVE_TO_INTAKE;
        }

        // Start the intake wheels
        armSubsystem.setIntakeSpeed(ArmConstants.INTAKE_SPEED);
    }

    @Override
    public void execute() {

        // If there is a note detected, then there is nothing to do
        if (armSubsystem.isNoteDetected()) {
            return;
        }

        // Get the current angles
        double currentLinkAngle = armSubsystem.getLinkAngle();
        double currentAimAngle  = armSubsystem.getAimAngle();

        double linkAngleError   = 0;
        double aimAngleError    = 0;

        double linkSpeed        = 0;
        double aimSpeed         = 0;

        switch (state) {

        case MOVE_AIM_ABOVE_90:

            // Run the aim motor to lift/extend the aim
            // while holding the link angle fixed.
            armSubsystem.setAimPivotSpeed(ArmConstants.FAST_AIM_SPEED);

            // TODO: Does this hold the link angle or is a PID required?
            armSubsystem.setLinkPivotSpeed(0);

            if (currentAimAngle > 90) {
                logStateTransition("Move to over bumper", "Aim above 90 deg");
                state = State.MOVE_TO_OVER_BUMPER;
            }

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

            // If past the bumper danger, move to the intake position.

            // If the aim is higher than the over-the-bumper angle, then it is safe to start
            // lowering the link to the intake position.
            if (aimAngleError < 0) {
                logStateTransition("Move to intake", "Aim extended past over bumper position");
                state = State.MOVE_TO_INTAKE;
            }

            break;

        case MOVE_TO_INTAKE:

            // Calculate the errors

            linkAngleError = ArmConstants.INTAKE_ARM_POSITION.linkAngle - currentLinkAngle;
            aimAngleError = ArmConstants.INTAKE_ARM_POSITION.aimAngle - currentAimAngle;

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

        }
    }

    @Override
    public boolean isFinished() {

        // If there is a note detected, then this command ends
        if (armSubsystem.isNoteDetected()) {
            setFinishReason("Note detected");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();

        logCommandEnd(interrupted);

        if (!interrupted) {
            CommandScheduler.getInstance().schedule(new CompactPoseCommand(armSubsystem));
        }
    }

}