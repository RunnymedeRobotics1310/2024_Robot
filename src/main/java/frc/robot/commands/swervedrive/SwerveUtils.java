package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.Constants.Swerve.Chassis.MIN_TRANSLATION_SPEED_MPS;

public class SwerveUtils {
    private SwerveUtils() {
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
    public static Rotation2d getHeadingToFieldPosition(Translation2d currentRobotLocation, Translation2d target) {
        Translation2d delta = target.minus(currentRobotLocation);
        return delta.getAngle();
    }

    /**
     * Returns true when the robot heading is within ROTATION_TOLERANCE_RADIANS of the desired
     * location
     *
     * @param currentHeading the current heading of the robot from the pose
     * @param desiredHeading the heading you would like to face
     * @param tolerance - the tolerance to use for the comparison, or null, which uses the default.
     */
    public static boolean isCloseEnough(Rotation2d currentHeading, Rotation2d desiredHeading, Rotation2d tolerance) {

        if (tolerance == null) {
            tolerance = ROTATION_TOLERANCE;
        }
        else if (tolerance.getRadians() < ROTATION_TOLERANCE.getRadians()) {
            // tolerance can't be below the minimum the robot can achieve
            tolerance = ROTATION_TOLERANCE;
        }
        Rotation2d delta = desiredHeading.minus(currentHeading);


        return Math.abs(delta.getRadians()) <= tolerance.getRadians();

    }

    /**
     * Returns true when the robot is located within TRANSLATION_TOLERANCE_METRES of the desired
     * location
     */
    public static boolean isCloseEnough(Translation2d currentLocation, Translation2d desiredLocation) {
        Translation2d delta = desiredLocation.minus(currentLocation);
        return Math.abs(delta.getNorm()) <= TRANSLATION_TOLERANCE_METRES;
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
    static Translation2d computeVelocity(Translation2d translationToTravel, double maxSpeed) {

        double distanceMetres = translationToTravel.getNorm();

        // don't worry about tiny translations
        if (distanceMetres < TRANSLATION_TOLERANCE_METRES) {
            return new Translation2d();
        }

        // safety code
        if (maxSpeed > MAX_TRANSLATION_SPEED_MPS) {
            maxSpeed = MAX_TRANSLATION_SPEED_MPS;
        }

        // ensure that we have enough room to decelerate
        double decelDistance  = DECEL_FROM_MAX_TO_STOP_DIST_METRES;
        double decelDistRatio = distanceMetres / decelDistance;
        if (decelDistRatio < 1) {
            maxSpeed *= decelDistRatio;
        }


        double speed;
        if (distanceMetres >= decelDistance) {
            // cruising
            speed = maxSpeed;
        }
        else {
            // decelerating
            double pctToGo = distanceMetres / decelDistance;
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
     * Utility function to compute the required rotation speed of the robot given the heading
     * provided. Uses a PID controller to compute the offset.
     *
     * @param target the desired heading of the robot
     * @param current the current heading of the robot
     * @return The required rotation speed of the robot
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     */
    static Rotation2d computeOmega(Rotation2d target, Rotation2d current) {

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
}
