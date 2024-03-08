package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Core swerve drive telemetry. IMPORTANT - the swerve variables here need to be set for
 * visualization to work. The names of the variables are important and should not be changed.
 */
public class SwerveCore {
    /** The number of swerve modules */
    public int      moduleCount           = -1310;
    /** The number of swerve modules */
    public double[] wheelLocations;
    /**
     * An array of rotation and velocity values describing the measured state of each swerve module
     */
    public double[] measuredStates;
    /**
     * An array of rotation and velocity values describing the desired state of each swerve module
     */
    public double[] desiredStates;
    /** The robot's current rotation based on odometry or gyro readings */
    public double   robotRotation         = -1310.0;
    /** The maximum achievable speed of the modules, used to adjust the size of the vectors. */
    public double   maxSpeed              = -1310.0;
    /** The units of the module rotations and robot rotation */
    public String   rotationUnit          = "degrees";
    /** The distance between the left and right modules. */
    public double   sizeLeftRight         = -1310.0;
    /** The distance between the front and back modules. */
    public double   sizeFrontBack         = -1310.0;
    /**
     * The direction the robot should be facing when the "Robot Rotation" is zero or blank. This
     * option is often useful to align with odometry data or match videos. 'up', 'right', 'down' or
     * 'left'
     */
    public String   forwardDirection      = "up";
    /**
     * The maximum achievable angular velocity of the robot. This is used to visualize the angular
     * velocity from the chassis speeds properties.
     */
    public double   maxAngularVelocity    = -1310.0;
    /**
     * The maximum achievable angular velocity of the robot. This is used to visualize the angular
     * velocity from the chassis speeds properties.
     */
    public double[] measuredChassisSpeeds = new double[3];
    /** Describes the desired forward, sideways and angular velocity of the robot. */
    public double[] desiredChassisSpeeds  = new double[3];

    /** Upload data to smartdashboard */
    public void post() {
        SmartDashboard.putNumber("swerve/moduleCount", moduleCount);
        SmartDashboard.putNumberArray("swerve/wheelLocations", wheelLocations == null ? new double[0] : wheelLocations);
        SmartDashboard.putNumberArray("swerve/measuredStates", measuredStates == null ? new double[0] : measuredStates);
        SmartDashboard.putNumberArray("swerve/desiredStates", desiredStates == null ? new double[0] : desiredStates);
        SmartDashboard.putNumber("swerve/robotRotation", robotRotation);
        SmartDashboard.putNumber("swerve/maxSpeed", maxSpeed);
        SmartDashboard.putString("swerve/rotationUnit", rotationUnit);
        SmartDashboard.putNumber("swerve/sizeLeftRight", sizeLeftRight);
        SmartDashboard.putNumber("swerve/sizeFrontBack", sizeFrontBack);
        SmartDashboard.putString("swerve/forwardDirection", forwardDirection);
        SmartDashboard.putNumber("swerve/maxAngularVelocity", maxAngularVelocity);
        SmartDashboard.putNumberArray("swerve/measuredChassisSpeeds", measuredChassisSpeeds);
        SmartDashboard.putNumberArray("swerve/desiredChassisSpeeds", desiredChassisSpeeds);
    }
}
