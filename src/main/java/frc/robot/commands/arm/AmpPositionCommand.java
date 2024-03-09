package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// Move arm to amp shoot pose
// Set shooter speed
public class AmpPositionCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_AMP, MOVE_TO_UNLOCK, MOVE_TO_OVER_BUMPER, SET_SHOOTER_SPEED
    };

    private State state = State.MOVE_TO_AMP;

    public AmpPositionCommand(ArmSubsystem armSubsystem) {

        super(armSubsystem);
    }

    @Override
    public void initialize() {

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
                // Move to and keep the position with a tolerance of 5 deg
                atArmAngle = this.driveToArmPosition(ArmConstants.SHOOT_AMP_ARM_POSITION, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
                        ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);
                break;

            case MOVE_TO_OVER_BUMPER:
                // Move to the requested angle with a tolerance of 5 deg
                atArmAngle = this.driveThroughArmPosition(ArmConstants.OVER_BUMPER_POSITION, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
                        ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

                // If the aim is higher than the over-the-bumper angle, then it is safe to start
                // raising the link to the amp position.
                if (atArmAngle) {
                    logStateTransition(State.MOVE_TO_AMP.name(), "Arm at over bumper position");
                    state = State.MOVE_TO_AMP;
                }
                break;

            case MOVE_TO_UNLOCK:
                // Move to the requested angle with a tolerance of 5 deg
                atArmAngle = this.driveThroughArmPosition(ArmConstants.UNLOCK_POSITION, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
                        ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

                // If past the bumper danger, move to the compact position.
                if (atArmAngle) {
                    logStateTransition(State.MOVE_TO_OVER_BUMPER.name(), "Arm at unlock position");
                    state = State.MOVE_TO_OVER_BUMPER;
                }
                break;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The command should hold the arm in the right place.  Should be interrupted.
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // run if interupted
        if (interrupted) {
            logCommandEnd(interrupted);
            armSubsystem.stop();
        }
    }
}