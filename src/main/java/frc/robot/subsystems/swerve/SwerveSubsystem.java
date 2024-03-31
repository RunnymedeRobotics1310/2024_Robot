package frc.robot.subsystems.swerve;

import static frc.robot.Constants.LightingConstants.VISPOSE1;
import static frc.robot.Constants.LightingConstants.VISPOSE2;
import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.RunnymedeUtils.format;
import static frc.robot.utils.vision.PoseConfidence.NONE;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.subsystems.RunnymedeSubsystemBase;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceHigh;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceLow;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceMedium;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceNone;
import frc.robot.telemetry.Telemetry;
import frc.robot.utils.vision.HughVision;
import frc.robot.utils.vision.InvalidVisionDataException;
import frc.robot.utils.vision.PoseConfidence;
import frc.robot.utils.vision.VisionPositionInfo;

public abstract class SwerveSubsystem extends RunnymedeSubsystemBase {

    private final SlewRateLimiter   xLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter   yLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter   omegaLimiter = new SlewRateLimiter(MAX_ROTATION_ACCELERATION_RAD_PER_SEC2);
    private final HughVision        hugh         = new HughVision();
    private final LightingSubsystem lighting;

    public SwerveSubsystem(LightingSubsystem lighting) {
        this.lighting = lighting;
    }

    /**
     * The primary method for controlling the drivebase. The provided {@link ChassisSpeeds}
     * specifies the robot-relative chassis speeds of the robot.
     * <p>
     * This method is responsible for applying safety code to prevent the robot from attempting to
     * exceed its physical limits both in terms of speed and acceleration.
     *
     * @param velocity The intended velocity of the robot chassis relative to itself.
     * @see ChassisSpeeds for how to construct a ChassisSpeeds object including
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     */
    public final void driveRobotOriented(ChassisSpeeds velocity) {

        double x = velocity.vxMetersPerSecond;
        double y = velocity.vyMetersPerSecond;
        double w = velocity.omegaRadiansPerSecond;

        // Limit change in values. Note this may not scale evenly - one may reach desired
        // speed before another. This will be corrected the next time drive() is called.

        x = xLimiter.calculate(x);
        y = yLimiter.calculate(y);
        w = omegaLimiter.calculate(w);

        ChassisSpeeds safeVelocity = new ChassisSpeeds(x, y, w);

        Telemetry.swerve.swerve_robot_chassis_speeds = safeVelocity;

        if (!Constants.Swerve.DISABLED) {
            driveRawRobotOriented(safeVelocity);
        }
    }

    /**
     * The internal method for controlling the drivebase. This code does not apply any
     * limiters or validation, and should be used by implementing swerve drive subsystems
     * only.
     * <p>
     * Takes the desired chassis speeds of the robot - in a robot-oriented configuration.
     *
     * @param velocity The intended velocity of the robot chassis relative to itself.
     * @see ChassisSpeeds for how to construct a ChassisSpeeds object including
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     */
    protected abstract void driveRawRobotOriented(ChassisSpeeds velocity);

    /**
     * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
     * field-oriented inputs into the required robot-oriented {@link ChassisSpeeds} object that can
     * be used by the robot.
     *
     * @param velocity the linear velocity of the robot in metres per second. Positive x is away
     * from the alliance wall, and positive y is toward the left wall when looking through the
     * driver station glass.
     * @param omega the rotation rate of the heading of the robot. CCW positive.
     * @see #driveRobotOriented(ChassisSpeeds)
     */
    public final void driveFieldOriented(Translation2d velocity, Rotation2d omega) {
        double     x     = velocity.getX();
        double     y     = velocity.getY();
        double     w     = omega.getRadians();
        Rotation2d theta = this.getPose().getRotation();
        Telemetry.swerve.swerve_velocity_field = velocity;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, w, theta);
        this.driveRobotOriented(chassisSpeeds);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public abstract Pose2d getPose();

    /**
     * Return the gyro rotation for the robot, with yaw adjusted for the configured offset
     *
     * @return adjusted rotation3d from the gyro
     */
    public abstract Rotation3d getGyroRotation3d();

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public abstract void zeroGyro();

    /**
     * Stop all motors as fast as possible
     */
    public void stop() {
        driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public abstract void lock();

    abstract public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs);

    /**
     * Updates the field relative position of the robot using module
     * position data from the modules themselves, plus the gyro.
     */
    protected abstract void updateOdometryWithStates();

    /**
     * Update the field relative position of the robot using vision
     * position data returned from the vision subsystem.
     */
    private void updateOdometryWithVisionInfo() {

        Pose2d             odometryPose = getPose();
        VisionPositionInfo visPosInfo;
        PoseConfidence     confidence;

        try {
            visPosInfo = hugh.getVisionPositionInfo(odometryPose);
            confidence = visPosInfo.confidence();
        }
        catch (InvalidVisionDataException e) {
            visPosInfo = null;
            confidence = NONE;
        }

        Telemetry.swerve.swerve_vispose = visPosInfo;
        if (confidence != NONE) {
            addVisionMeasurement(visPosInfo.pose(), visPosInfo.timestampSeconds(), visPosInfo.deviation());
        }

        switch (confidence) {
        case HIGH:
            lighting.setPattern(VISPOSE1, VisionConfidenceHigh.getInstance());
            lighting.setPattern(VISPOSE2, VisionConfidenceHigh.getInstance());
            break;
        case MEDIUM:
            lighting.setPattern(VISPOSE1, VisionConfidenceMedium.getInstance());
            lighting.setPattern(VISPOSE2, VisionConfidenceMedium.getInstance());
            break;
        case LOW:
            lighting.setPattern(VISPOSE1, VisionConfidenceLow.getInstance());
            lighting.setPattern(VISPOSE2, VisionConfidenceLow.getInstance());
            break;
        case NONE:
            lighting.setPattern(VISPOSE1, VisionConfidenceNone.getInstance());
            lighting.setPattern(VISPOSE2, VisionConfidenceNone.getInstance());
            break;
        }
    }

    public abstract void updateTelemetry();

    /**
     * Set the swerve module state for the specified module. This is intended to be used ONLY in
     * test mode!
     *
     * @param module the module configuration object - used to identify the module only.
     * @param desiredState the desired state of the swerve module
     */
    public abstract void setModuleStateForTestMode(Constants.Swerve.Module module, SwerveModuleState desiredState);

    public abstract void resetOdometry(Pose2d replacementPose);

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
     */
    public final Rotation2d getHeadingToFieldPosition(Translation2d target) {
        Translation2d current = getPose().getTranslation();
        Translation2d delta   = target.minus(current);
        return delta.getAngle();
    }


    /**
     * Drive as fast as safely possible to the specified pose, up ot the max speed specified.
     *
     * @param desiredPose the desired location on the field
     */
    public final void driveToFieldPose(Pose2d desiredPose, double maxSpeedMPS) {
        Pose2d        current  = getPose();
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

        driveFieldOriented(velocity, omega);
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


    public Rotation2d computeOmegaForOffset(Rotation2d offset) {
        return computeOmega(offset, new Rotation2d());
    }

    /**
     * Utility function to compute the required rotation speed of the robot given its current
     * heading. Uses a PID controller to compute the offset.
     *
     * @param target the desired heading of the robot
     * @return The required rotation speed of the robot
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     */
    public final Rotation2d computeOmega(Rotation2d target) {
        return computeOmega(target, getPose().getRotation());
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
     * Return the distance in metres to the specified field position
     */
    public final double distanceToFieldPosition(Translation2d target) {
        return getPose().getTranslation().getDistance(target);
    }

    /**
     * Returns true when the robot is located within TRANSLATION_TOLERANCE_METRES of the desired
     * location
     */
    public final boolean isCloseEnough(Translation2d desiredLocation) {
        Translation2d delta = desiredLocation.minus(getPose().getTranslation());
        return Math.abs(delta.getNorm()) <= TRANSLATION_TOLERANCE_METRES;
    }

    /**
     * Returns true when the robot heading is within ROTATION_TOLERANCE_RADIANS of the desired
     * location
     */
    public final boolean isCloseEnough(Rotation2d desiredHeading) {
        Rotation2d delta = desiredHeading.minus(getPose().getRotation());
        return Math.abs(delta.getRadians()) <= ROTATION_TOLERANCE.getRadians();
    }

    public final boolean isCloseEnough(Pose2d desiredPose) {
        return isCloseEnough(desiredPose.getTranslation()) && isCloseEnough(desiredPose.getRotation());
    }

    @Override
    public void periodic() {
        super.periodic();
        updateOdometryWithStates();
        updateOdometryWithVisionInfo();
        updateTelemetry();
        Telemetry.swerve.swerve_pose = getPose();
    }

    @Override
    public String toString() {
        return "SwerveSubsystem Current Pose: " + format(getPose());
    }
}
