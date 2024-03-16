package frc.robot.commands.climb;

import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

abstract class BaseClimbCommand extends LoggingCommand {

    protected final ClimbSubsystem  climbSubsystem;
    protected final SwerveSubsystem driveSubsystem;

    protected BaseClimbCommand(ClimbSubsystem climb, SwerveSubsystem drive) {
        this.climbSubsystem = climb;
        this.driveSubsystem = drive;
        addRequirements(climb);
    }

    protected void climbFlat(double speed) {
        if (speed > 0) {
            // going up only.
            return;
        }

        double left        = speed;
        double right       = speed;
        double rollRadians = driveSubsystem.getGyroRotation3d().getZ();

        if (Math.abs(rollRadians) > Constants.ClimbConstants.LEVEL_CLIMB_TOLERANCE.getRadians()) {
            if (rollRadians > 0) {
                // right is too low
                if (climbSubsystem.rightAllTheWayDown()) {
                    right = 0;
                }
                else {
                    right = 0.5 * speed;
                }
            }
            else {
                // right is too high
                if (climbSubsystem.leftAllTheWayDown()) {
                    left = 0;
                }
                else {
                    left = 0.5 * speed;
                }
            }
        }

        climbSubsystem.setClimbSpeeds(left, right);
    }

}
