package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Move arm to speaker shoot pose
// Set shooter speed (distance based)
public class CompactFromIntakeCommand extends ArmBaseCommand {

    private enum State {
        LIFT_LINK_10_DEG, MOVE_BOTH, LOCK, LOCKED
    };

    private State state = State.LIFT_LINK_10_DEG;

    private boolean tighterTolerance = false;

    public CompactFromIntakeCommand(ArmSubsystem armSubsystem, boolean tighterTolerance) {
        super(armSubsystem);
        this.tighterTolerance = tighterTolerance;
    }

    @Override
    public void initialize() {

        logCommandStart();

        double tolerance = tighterTolerance ? 2 : 4;

        // If we are close to the locked position, there is nothing to do.
        if (isAtArmPosition(ArmConstants.COMPACT_ARM_POSITION, tolerance)) {

            if (armSubsystem.getAimAngle() <= ArmConstants.COMPACT_ARM_POSITION.aimAngle
                    && armSubsystem.getLinkAngle() >= ArmConstants.COMPACT_ARM_POSITION.linkAngle) {
                setStateAndLog(State.LOCK, "Arm already near compact.  Lock it");
            }
            else {
                setStateAndLog(State.LOCKED, "Arm already in locked compact");
            }
        }
        else {
            setStateAndLog(State.LIFT_LINK_10_DEG, "Link is low, lift before adjusting aim");
        }

        // Stop all arm motors to turn off the shooter and intake.
        armSubsystem.stop();
    }

    @Override
    public void execute() {

        double linkSpeed = 0;
        double aimSpeed  = 0;

        switch (state) {

            case LIFT_LINK_10_DEG:

                armSubsystem.setLinkPivotSpeed(.5);

                if (armSubsystem.getLinkAngle() > ArmConstants.INTAKE_ARM_POSITION.linkAngle + 4) {
                    setStateAndLog(State.MOVE_BOTH, "Link lifted, Link at" + armSubsystem.getLinkAngle());
                }
                break;

            case MOVE_BOTH:

                linkSpeed = .9;
                aimSpeed = -.6;

                if (armSubsystem.getLinkAngle() > ArmConstants.COMPACT_ARM_POSITION.linkAngle) {

                    if (armSubsystem.getLinkAngle() > ArmConstants.COMPACT_ARM_POSITION.linkAngle + 5) {
                        linkSpeed = -.1;
                    }
                    else {
                        linkSpeed = 0;
                    }
                }

                if (armSubsystem.getAimAngle() < ArmConstants.COMPACT_ARM_POSITION.aimAngle) {
                    aimSpeed = 0;
                }

                armSubsystem.setLinkPivotSpeed(linkSpeed);
                armSubsystem.setAimPivotSpeed(aimSpeed);

                if (armSubsystem.getLinkAngle() > ArmConstants.COMPACT_ARM_POSITION.linkAngle
                        && armSubsystem.getAimAngle() < ArmConstants.COMPACT_ARM_POSITION.aimAngle) {

                    setStateAndLog(State.LOCK, "In Position - let's lock. Link" + armSubsystem.getLinkAngle() + ", Aim " + armSubsystem.getAimAngle());
                }
                break;

            case LOCK:

                armSubsystem.setLinkPivotSpeed(-.2);
                armSubsystem.setAimPivotSpeed(0);

                // If past the bumper danger, move to the intake position.
                if (this.isStateTimeoutExceeded(.2)) {

                    armSubsystem.setLinkPivotSpeed(0);

                    setStateAndLog(State.LOCKED, "In locked position");
                }

                break;

            case LOCKED:
                // Nothing to do here
                break;
        }
    }

    @Override
    public boolean isFinished() {

        // If the arm is in the locked position
        if (state == State.LOCKED) {
            setFinishReason("Arm Locked in Compact Position");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();

        logCommandEnd(interrupted);
    }

    /**
     * Set the state and log the transition
     *
     * @param newState State to transition to
     * @param reason Reason for the transition for logging
     */
    private void setStateAndLog(CompactFromIntakeCommand.State newState, String reason) {
        logStateTransition(newState.name(), reason);
        state = newState;
    }

}