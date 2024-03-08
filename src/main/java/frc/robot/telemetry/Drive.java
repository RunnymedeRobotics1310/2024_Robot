package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.vision.VisionPositionInfo;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class Drive {
    Drive() {
    }

    public double        teleop_correctedHeadingDeg  = -1310.0;
    public double        teleop_vX                   = -1310.0;
    public double        teleop_vY                   = -1310.0;
    public double        teleop_ccwRotAngularVelPct  = -1310.0;
    public double        teleop_rawDesiredHeadingDeg = -1310.0;
    public double        teleop_boostFactor          = -1310.0;
    public String        teleop_mode                 = null;
    public boolean       teleop_lockOnSpeaker        = false;
    public Translation2d teleop_velocity             = null;
    public Rotation2d    teleop_theta                = null;
    public Rotation2d    teleop_omega                = null;
    public Transform2d   drive_to_pose_delta         = null;
    public Pose2d        drive_to_pose_desired       = null;
    public Translation2d drive_to_pose_velocity      = null;
    public Rotation2d    drive_to_pose_omega         = null;

    void post() {

        // Teleop
        SmartDashboard.putString("Drive/Teleop/Alliance", getRunnymedeAlliance().name());

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
