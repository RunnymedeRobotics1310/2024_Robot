package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.RunnymedeUtils.format;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Telemetry;

public abstract class BaseDriveCommand extends LoggingCommand {
    protected final SwerveSubsystem swerve;

    public BaseDriveCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    /**
     * Utility function to compute the required rotation speed of the robot given its current
     * heading. Uses a PID controller to compute the offset.
     *
     * @param target the desired heading of the robot
     * @return The required rotation speed of the robot
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     */
    protected final Rotation2d computeOmega(Rotation2d target) {
        return SwerveUtils.computeOmega(target, swerve.getPose().getRotation());
    }

    public Rotation2d computeOmegaForOffset(Rotation2d offset) {
        return SwerveUtils.computeOmega(offset, new Rotation2d());
    }



    /**
     * Drive as fast as safely possible to the specified pose, up ot the max speed specified.
     *
     * @param desiredPose the desired location on the field
     */
    protected final void driveToFieldPose(Pose2d desiredPose, double maxSpeedMPS) {
        Pose2d        current  = swerve.getPose();
        Transform2d   delta    = RunnymedeUtils.difference(desiredPose, current);

        Translation2d velocity = SwerveUtils.computeVelocity(delta.getTranslation(), maxSpeedMPS);
        Rotation2d    omega    = computeOmega(desiredPose.getRotation());

        log("Current: " + format(current)
            + " Delta: " + format(delta.getTranslation()) + " m @ " + format(delta.getRotation())
            + " Target: " + format(desiredPose)
            + " Velocity: " + format(velocity) + "m/s @ " + format(omega) + "/s");

        Telemetry.drive.drive_to_pose_delta    = delta;
        Telemetry.drive.drive_to_pose_desired  = desiredPose;
        Telemetry.drive.drive_to_pose_velocity = velocity;
        Telemetry.drive.drive_to_pose_omega    = omega;

        swerve.driveFieldOriented(velocity, omega);
    }

    /**
     * Return the distance in metres to the specified field position
     */
    protected final double distanceToFieldPosition(Translation2d target) {
        return swerve.getPose().getTranslation().getDistance(target);
    }

    /**
     * Returns true when the robot is located within TRANSLATION_TOLERANCE_METRES of the desired
     * location
     */
    protected final boolean isCloseEnough(Translation2d desiredLocation) {
        return SwerveUtils.isCloseEnough(swerve.getPose().getTranslation(), desiredLocation);
    }



    /**
     * Returns true when the robot heading is within ROTATION_TOLERANCE_RADIANS of the desired
     * location
     */
    protected final boolean isCloseEnough(Rotation2d desiredHeading) {
        return isCloseEnough(desiredHeading, ROTATION_TOLERANCE);
    }



    /**
     * Returns true when the robot heading is within the specified tolerance of the desired
     * location
     */
    protected final boolean isCloseEnough(Rotation2d desiredHeading, Rotation2d tolerance) {
        return SwerveUtils.isCloseEnough(swerve.getPose().getRotation(), desiredHeading, tolerance);
    }

    protected final boolean isCloseEnough(Pose2d desiredPose) {
        return isCloseEnough(desiredPose.getTranslation()) && isCloseEnough(desiredPose.getRotation());
    }

    /**
     * Compute the heading required to face the specified position on the field.
     * <p>
     * This handy utility enables the user to specify a field position, get
     * the required headiang for it, and pass it into computeOmega to
     * determine the required omega to face that position - even as the robot
     * moves across the field.
     * <code>
     *     Rotation2d heading getHeadingToFieldPosition(speaker)
     *     Rotation2d omega computeOmega(heading)
     * </code>
     * <p>
     * In other words, this utility enables the user to "lock on a target".
     *
     * @param target field position
     * @return the heading toward that position.
     * @see #computeOmega(Rotation2d)
     */
    protected final Rotation2d getHeadingToFieldPosition(Translation2d target) {
        return SwerveUtils.getHeadingToFieldPosition(swerve.getPose().getTranslation(), target);
    }


}