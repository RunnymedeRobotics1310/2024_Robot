package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.vision.VisionPositionInfo;

import java.util.LinkedHashMap;
import java.util.Map;

public class SwervePlus {

    SwervePlus() {
    }

    public double                              rawImuDegrees;
    public double                              adjustedImuDegrees;

    public static Map<String, ModuleTelemetry> modules = new LinkedHashMap<>();

    public ModuleTelemetry getModule(String name) {
        if (!modules.containsKey(name)) {
            modules.put(name, new ModuleTelemetry());
        }
        return modules.get(name);
    }

    public static class ModuleTelemetry {
        ModuleTelemetry() {
        }

        public double     speedMetersPerSecond;
        public double     angleDegrees;
        public double     absoluteEncoderPositionDegrees;
        public Rotation2d angleMotorPosition;
        public double     driveMotorPosition;
    }

    public ChassisSpeeds      swerve_robot_chassis_speeds = null;
    public Translation2d      swerve_velocity_field       = null;
    public VisionPositionInfo swerve_vispose              = null;
    public Pose2d             swerve_pose                 = null;


    void post() {
        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/rawImuDegrees", String.format("%.2f", rawImuDegrees));
        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/adjustedImuDegrees", String.format("%.2f", adjustedImuDegrees));

        for (String name : modules.keySet()) {
            ModuleTelemetry module = modules.get(name);
            SmartDashboard.putNumber(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Speed Setpoint",
                module.speedMetersPerSecond);
            SmartDashboard.putNumber(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Angle Setpoint", module.angleDegrees);

            SmartDashboard.putNumber(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Absolute Encoder Position Degrees",
                module.absoluteEncoderPositionDegrees);
            SmartDashboard.putString(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Angle Position",
                module.angleMotorPosition.toString());
            SmartDashboard.putNumber(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Drive Position", module.driveMotorPosition);


        }

        // Swerve
        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/robot_chassis_speed",
            swerve_robot_chassis_speeds == null ? ""
                : String.format("%.2f,%.2f m/s %.0f deg/s)",
                    swerve_robot_chassis_speeds.vxMetersPerSecond, swerve_robot_chassis_speeds.vyMetersPerSecond,
                    Rotation2d.fromRadians(swerve_robot_chassis_speeds.omegaRadiansPerSecond).getDegrees()));

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/robot_speed", swerve_robot_chassis_speeds == null ? ""
            : String.format("%.2f m/s",
                Math.hypot(swerve_robot_chassis_speeds.vxMetersPerSecond, swerve_robot_chassis_speeds.vyMetersPerSecond)));

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/velocity_field",
            LoggingCommand.format(swerve_velocity_field) + " m/s");

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/vispose", swerve_vispose == null ? "" : swerve_vispose.toString());

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/location", swerve_pose == null ? ""
            : String.format("%.2f,%.2f m", swerve_pose.getTranslation().getX(), swerve_pose.getTranslation().getY()));

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/heading",
            swerve_pose == null ? "" : String.format("%.0f deg", swerve_pose.getRotation().getDegrees()));

    }
}
