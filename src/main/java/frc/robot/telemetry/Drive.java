package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.vision.VisionPositionInfo;

public class Drive {
    Drive() {
    }

    public double             teleop_correctedHeadingDeg  = -1310.0;
    public double             teleop_vX                   = -1310.0;
    public double             teleop_vY                   = -1310.0;
    public double             teleop_ccwRotAngularVelPct  = -1310.0;
    public double             teleop_rawDesiredHeadingDeg = -1310.0;
    public double             teleop_boostFactor          = -1310.0;
    public String             teleop_mode                 = null;
    public boolean            teleop_lockOnSpeaker        = false;
    public Translation2d      teleop_velocity             = null;
    public Rotation2d         teleop_theta                = null;
    public Rotation2d         teleop_omega                = null;

    public ChassisSpeeds      swerve_robot_chassis_speeds = null;
    public Translation2d      swerve_velocity_field       = null;
    public VisionPositionInfo swerve_vispose              = null;
    public Pose2d             swerve_pose                 = null;

    public Transform2d        drive_to_pose_delta         = null;
    public Pose2d             drive_to_pose_desired       = null;
    public Translation2d      drive_to_pose_velocity      = null;
    public Rotation2d         drive_to_pose_omega         = null;



    void post() {

        // Teleop

        SmartDashboard.putNumber("Drive/Teleop/correctedHeadingDeg", teleop_correctedHeadingDeg);
        SmartDashboard.putNumber("Drive/Teleop/vX", teleop_vX);
        SmartDashboard.putNumber("Drive/Teleop/vY", teleop_vY);
        SmartDashboard.putNumber("Drive/Teleop/ccwRotAngularVelPct", teleop_ccwRotAngularVelPct);
        SmartDashboard.putNumber("Drive/Teleop/rawDesiredHeadingDeg", teleop_rawDesiredHeadingDeg);
        SmartDashboard.putNumber("Drive/Teleop/boostFactor", teleop_boostFactor);
        SmartDashboard.putString("Drive/Teleop/mode", teleop_mode == null ? "" : teleop_mode);
        SmartDashboard.putBoolean("Drive/Teleop/lockOnSpeaker", teleop_lockOnSpeaker);

        SmartDashboard.putString("Drive/Teleop/velocity",
            teleop_velocity == null ? ""
                : String.format("%.2f", teleop_velocity.getNorm()) + "m/s at "
                    + String.format("%.2f", teleop_velocity.getAngle().getDegrees()) + "deg");

        SmartDashboard.putString("Drive/Teleop/theta ",
            teleop_theta == null ? "" : String.format("%.2f", teleop_theta.getDegrees()) + "deg");

        SmartDashboard.putString("Drive/Teleop/omega",
            teleop_omega == null ? "" : String.format("%.2f", teleop_omega.getDegrees()) + "deg/s");


        // Swerve

        SmartDashboard.putString("Drive/Swerve/robot_chassis_speed",
            swerve_robot_chassis_speeds == null ? ""
                : String.format("%.2f,%.2f m/s %.0f deg/s)",
                    swerve_robot_chassis_speeds.vxMetersPerSecond, swerve_robot_chassis_speeds.vyMetersPerSecond,
                    Rotation2d.fromRadians(swerve_robot_chassis_speeds.omegaRadiansPerSecond).getDegrees()));

        SmartDashboard.putString("Drive/Swerve/robot_speed", swerve_robot_chassis_speeds == null ? ""
            : String.format("%.2f m/s",
                Math.hypot(swerve_robot_chassis_speeds.vxMetersPerSecond, swerve_robot_chassis_speeds.vyMetersPerSecond)));

        SmartDashboard.putString("Drive/Swerve/velocity_field", LoggingCommand.format(swerve_velocity_field) + " m/s");

        SmartDashboard.putString("Drive/Swerve/vispose", swerve_vispose == null ? "" : swerve_vispose.toString());

        SmartDashboard.putString("Drive/Swerve/location", swerve_pose == null ? ""
            : String.format("%.2f,%.2f m", swerve_pose.getTranslation().getX(), swerve_pose.getTranslation().getY()));

        SmartDashboard.putString("Drive/Swerve/heading",
            swerve_pose == null ? "" : String.format("%.0f deg", swerve_pose.getRotation().getDegrees()));

        // drive to position
        SmartDashboard.putString("Drive/ToFieldPosition/delta",
            drive_to_pose_delta == null ? ""
                : LoggingCommand.format(drive_to_pose_delta.getTranslation()) + " m @ "
                    + LoggingCommand.format(drive_to_pose_delta.getRotation()));

        SmartDashboard.putString("Drive/ToFieldPosition/desired", LoggingCommand.format(drive_to_pose_desired));

        SmartDashboard.putString("Drive/ToFieldPosition/velocity",
            LoggingCommand.format(drive_to_pose_velocity) + "m/s @ " + LoggingCommand.format(drive_to_pose_omega) + "/s");


    }
}
