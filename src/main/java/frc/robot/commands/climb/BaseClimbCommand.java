package frc.robot.commands.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.ClimbConstants.MAX_ROBOT_LIFT_SPEED;
import static frc.robot.Constants.ClimbConstants.MIN_LOWER_CLIMBERS_SPEED;

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

        boolean leftDown  = climbSubsystem.leftAllTheWayDown();
        boolean rightDown = climbSubsystem.rightAllTheWayDown();

        double  left      = speed;
        double  right     = speed;

        if (leftDown && rightDown) {
            // we went too far - stop.
            left  = 0;
            right = 0;
        }
        else {
            // figure out how crooken we are
            double rollRadians    = driveSubsystem.getGyroRotation3d().getZ();
            double absRollRadians = Math.abs(rollRadians);

            if (absRollRadians > Constants.ClimbConstants.LEVEL_CLIMB_TOLERANCE.getRadians()) {
                // crooked. Slow down the side that is too low.
                if (rollRadians > 0) {
                    // right is too low - slow down the left side
                    if (leftDown) {
                        // it's all the way down, don't move it
                        left = 0;
                    }
                    else {
                        // slow it down
                        left = getSpeedForLevelError(absRollRadians);
                    }
                }
                else {
                    // left side is too low
                    if (rightDown) {
                        right = 0;
                    }
                    else {
                        right = getSpeedForLevelError(absRollRadians);
                    }
                }
            }
            else {
                // close enough
                left  = 0;
                right = 0;
            }
        }

        climbSubsystem.setClimbSpeeds(left, right);
    }

    private static double getSpeedForLevelError(double errorRadians) {
        // we will reduce the speed only in the range where the speed is enough to move the climbers
        double speedRange = MAX_ROBOT_LIFT_SPEED - MIN_LOWER_CLIMBERS_SPEED;
        // error will be at MOST 1/2 rotation so get a factor approximately between 0 and 1
        double factor = Rotation2d.fromRadians(errorRadians).getRotations() * 2;
        // return the minimum speed + a value dependent on how far off it is
        return MIN_LOWER_CLIMBERS_SPEED + speedRange * factor;
    }

}
