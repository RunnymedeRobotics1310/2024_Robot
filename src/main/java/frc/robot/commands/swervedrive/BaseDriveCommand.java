package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.RunnymedeUtils.format;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Swerve.Chassis.VelocityPIDConfig;
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
        return computeOmega(target, swerve.getPose().getRotation());
    }

    public Rotation2d computeOmegaForOffset(Rotation2d offset) {
        return computeOmega(offset, new Rotation2d());
    }

    /**
     * Utility function to compute the required rotation speed of the robot given the heading
     * provided. Uses a PID controller to compute the offset.
     *
     * @param target the desired heading of the robot
     * @param current the current heading of the robot
     * @return The required rotation speed of the robot
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     */
    private static Rotation2d computeOmega(Rotation2d target, Rotation2d current) {

        double targetRad  = normalizeRotation(target.getRadians());
        double currentRad = normalizeRotation(current.getRadians());

        double errorRad   = targetRad - currentRad;
        errorRad = normalizeRotation(errorRad);
        double       absErrRad = Math.abs(errorRad);
        double       errSignum = Math.signum(errorRad);

        final double omegaRad;
        if (absErrRad < ROTATION_TOLERANCE.getRadians()) {
            omegaRad = 0;
        }
        else if (absErrRad < ROTATION_SLOW_ZONE.getRadians()) {
            omegaRad = errSignum * MIN_ROTATIONAL_VELOCITY_PER_SEC.getRadians();
        }
        else {
            omegaRad = errSignum * MAX_ROTATIONAL_JUMP_VELOCITY_PER_SEC.getRadians();
        }

        // log(String.format("omega: %.2f", omegaRad));

        return Rotation2d.fromRadians(omegaRad);
    }

    /**
     * Ensure that rotation error is between -pi and pi radians.
     */
    private static double normalizeRotation(double radians) {

        radians = radians % (2 * Math.PI);

        if (radians > Math.PI) {
            radians -= (2 * Math.PI);
        }
        else if (radians < -Math.PI) {
            radians += (2 * Math.PI);
        }

        return radians;
    }

    /**
     * Return a velocity that will traverse the specified translation as fast as possible without
     * overshooting the location. The initial speed is expected to be 0 and the final speed is
     * expected to be 0.
     *
     * @param translationToTravel the desired translation to travel
     * @param maxSpeed the maximum speed to travel in Metres per Second
     * @return the velocity vector, in metres per second that the robot can safely travel
     * to traverse the distance specified
     */
    private static Translation2d computeVelocity(Translation2d translationToTravel, double maxSpeed) {

        double distanceMetres = translationToTravel.getNorm();
        double absDistMetres  = Math.abs(distanceMetres);

        // don't worry about tiny translations
        if (absDistMetres < TRANSLATION_TOLERANCE_METRES) {
            return new Translation2d();
        }

        // safety code
        if (maxSpeed > MAX_TRANSLATION_SPEED_MPS) {
            maxSpeed = MAX_TRANSLATION_SPEED_MPS;
        }

        // ensure that we have enough room to decelerate
        double decelDistance  = DECEL_FROM_MAX_TO_STOP_DIST_METRES;
        double decelDistRatio = absDistMetres / decelDistance;
        if (decelDistRatio < 1) {
            maxSpeed *= decelDistRatio;
        }


        double speed;
        if (absDistMetres >= decelDistance) {
            // cruising
            speed = maxSpeed;
        }
        else {
            // decelerating
            double pctToGo = absDistMetres / decelDistance;
            speed = maxSpeed * pctToGo * VelocityPIDConfig.P;
        }

        // Confirm speed is not too slow to move
        if (speed < MIN_TRANSLATION_SPEED_MPS) {
            speed = MIN_TRANSLATION_SPEED_MPS;
        }


        Rotation2d angle = translationToTravel.getAngle();

        double     xSign = Math.signum(translationToTravel.getX());
        double     ySign = Math.signum(translationToTravel.getY());
        return new Translation2d(xSign * speed * Math.abs(angle.getCos()), ySign * speed * Math.abs(angle.getSin()));
    }

    /**
     * Drive as fast as safely possible to the specified pose, up ot the max speed specified.
     *
     * @param desiredPose the desired location on the field
     */
    protected final void driveToFieldPose(Pose2d desiredPose, double maxSpeedMPS) {
        Pose2d        current  = swerve.getPose();
        Transform2d   delta    = RunnymedeUtils.difference(desiredPose, current);

        Translation2d velocity = computeVelocity(delta.getTranslation(), maxSpeedMPS);
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
        Translation2d delta = desiredLocation.minus(swerve.getPose().getTranslation());
        return Math.abs(delta.getNorm()) <= TRANSLATION_TOLERANCE_METRES;
    }

    /**
     * Returns true when the robot heading is within ROTATION_TOLERANCE_RADIANS of the desired
     * location
     */
    protected final boolean isCloseEnough(Rotation2d desiredHeading) {
        Rotation2d delta = desiredHeading.minus(swerve.getPose().getRotation());
        return Math.abs(delta.getRadians()) <= ROTATION_TOLERANCE.getRadians();
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
        Translation2d current = swerve.getPose().getTranslation();
        Translation2d delta   = target.minus(current);
        return delta.getAngle();
    }
}