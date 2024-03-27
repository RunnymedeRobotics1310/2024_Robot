package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.vision.PoseConfidence;

import java.util.Arrays;

public class Hugh {
    Hugh() {
    }

    public Constants.BotTarget botTarget              = null;
    public double              priorityId             = -1310.0;
    public boolean             targetFound            = false;
    public double              tid                    = -1310.0;
    public double              tx                     = -1310.0;
    public double              ty                     = -1310.0;
    public double              ta                     = -1310.0;
    public double              tl                     = -1310.0;
    public double[]            botpose                = null;
    public double              targetAvgDist          = -1310.0;
    public int                 numTags                = -1310;
    public double              distanceToTargetMetres = -1310.0;
    public boolean             isAlignedWithTarget    = false;
    public Rotation2d          targetOffset           = null;
    public String              aprilTagInfo           = null;
    public double              shooterAngle           = Double.MIN_VALUE;
    public boolean             poseUpdate             = false;
    public PoseConfidence      poseConfidence         = PoseConfidence.NONE;

    void post() {
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/BotTarget", botTarget.toString());
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/PriorityId", "" + priorityId);
        // SmartDashboard.putBoolean(Telemetry.PREFIX + "VisionHugh/Target Found", targetFound);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tid", tid);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tx", tx);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/ty", ty);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/ta", ta);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tl", tl);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/Botpose", Arrays.toString(botpose));
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/TargetAvgDist", targetAvgDist);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/NumTags", "" + numTags);
        // SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/AprilTagInfo", aprilTagInfo);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/DistToTarget", distanceToTargetMetres);
        SmartDashboard.putBoolean(Telemetry.PREFIX + "VisionHugh/AlignedWithTarget", isAlignedWithTarget);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/TargetOffset",
            targetOffset == null ? "null" : targetOffset.toString());
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/shooterAngle", shooterAngle);
        SmartDashboard.putBoolean(Telemetry.PREFIX + "VisionHugh/PoseUpdate", poseUpdate);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/PoseConf", poseConfidence.name());
    }
}
