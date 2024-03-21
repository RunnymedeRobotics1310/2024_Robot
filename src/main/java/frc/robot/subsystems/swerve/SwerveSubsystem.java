package frc.robot.subsystems.swerve;

import static frc.robot.Constants.LightingConstants.VISPOSE1;
import static frc.robot.Constants.LightingConstants.VISPOSE2;
import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATION_ACCELERATION_RAD_PER_SEC2;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_ACCELERATION_MPS2;
import static frc.robot.Constants.VisionConstants.CAMERA_LOC_REL_TO_ROBOT_CENTER;
import static frc.robot.Constants.VisionConstants.getVisionStandardDeviation;
import static frc.robot.RunnymedeUtils.format;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.RunnymedeSubsystemBase;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.*;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.VisionPositionInfo;
import frc.robot.subsystems.vision.PoseConfidence;
import frc.robot.telemetry.Telemetry;

public abstract class SwerveSubsystem extends RunnymedeSubsystemBase {

    private final HughVisionSubsystem visionSubsystem;
    LightingSubsystem                 lightingSubsystem;
    private final SlewRateLimiter     xLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter     yLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter     omegaLimiter = new SlewRateLimiter(MAX_ROTATION_ACCELERATION_RAD_PER_SEC2);

    public SwerveSubsystem(HughVisionSubsystem visionSubsystem, LightingSubsystem lightingSubsystem) {
        this.visionSubsystem   = visionSubsystem;
        this.lightingSubsystem = lightingSubsystem;
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

    abstract protected void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs);

    /**
     * Updates the field relative position of the robot using module
     * position data from the modules themselves, plus the gyro.
     */
    protected abstract void updateOdometryWithStates();

    /**
     * Update the field relative position of the robot using vision
     * position data returned from the vision subsystem.
     *
     * @see frc.robot.Constants.VisionConstants#getVisionStandardDeviation(frc.robot.subsystems.vision.PoseConfidence,
     * double) fortuning info
     */
    private void updateOdometryWithVisionInfo() {
        VisionPositionInfo visPose = visionSubsystem.getPositionInfo();
        Telemetry.swerve.swerve_vispose = visPose;

        // ignore unreliable info from vision subsystem
        if (visPose == null) {
            updateLighting(null);
            return;
        }

        updateLighting(visPose.poseConfidence());

        Pose2d         odoPose = getPose();

        // how different is vision data from estimated data?
        double         delta_m = getPose().getTranslation().getDistance(visPose.pose().getTranslation());

        Matrix<N3, N1> stds    = getVisionStandardDeviation(visPose.poseConfidence(), delta_m);

        // ignore drastically different data
        if (stds == null) {
            updateLighting(null);
            return;
        }

        if (delta_m > 0.3) {
            log(String.format("Vision data is too different from odometry: %2f m", delta_m) + " odo: "
                + format(odoPose) + " vis: " + format(visPose.pose()));
        }

        double timeInSeconds = Timer.getFPGATimestamp() - (visPose.latencyMillis() / 1000);

        this.addVisionMeasurement(visPose.pose(), timeInSeconds, stds);
    }

    private void updateLighting(PoseConfidence confidence) {
        if (confidence == null) {
            lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceNone.getInstance());
            lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceNone.getInstance());
        }
        else {
            switch (confidence) {
            case HIGH:
                lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceHigh.getInstance());
                lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceHigh.getInstance());
                break;
            case MEDIUM:
                lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceMedium.getInstance());
                lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceMedium.getInstance());
                break;
            case LOW:
                lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceLow.getInstance());
                lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceLow.getInstance());
                break;
            case NONE:
                lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceNone.getInstance());
                lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceNone.getInstance());
                break;
            }
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

    @Override
    public void periodic() {
        super.periodic();
        updateOdometryWithStates();
        updateOdometryWithVisionInfo();
        updateTelemetry();
        Pose2d pose = getPose();
        Telemetry.swerve.swerve_pose = pose;
    }

    @Override
    public String toString() {
        return "SwerveSubsystem Current Pose: " + format(getPose());
    }
}
