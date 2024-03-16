package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.vision.PoseConfidence;
import frc.robot.subsystems.vision.VisionPositionInfo;

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
    public double[]            botpost                = null;
    public double              targetAvgDist          = -1310.0;
    public VisionPositionInfo  visPose                = null;
    public int                 numTags                = -1310;
    public double              distanceToTargetMetres = -1310.0;
    public boolean             isAlignedWithTarget    = false;
    public Rotation2d          targetOffset           = null;
    public String              aprilTagInfo           = null;
    public double               shooterAngle = Double.MIN_VALUE;

    void post() {
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/BotTarget", botTarget.toString());
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/PriorityId", "" + priorityId);
        // SmartDashboard.putBoolean(Telemetry.PREFIX + "VisionHugh/Target Found", targetFound);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tid", tid);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tx", tx);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/ty", ty);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/ta", ta);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tl", tl);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/Botpose", Arrays.toString(botpost));
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/TargetAvgDist", targetAvgDist);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/PoseConf",
            visPose == null ? PoseConfidence.NONE.toString() : visPose.poseConfidence().toString());
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/NumTags", "" + numTags);
        // SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/AprilTagInfo", aprilTagInfo);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/DistToTarget", distanceToTargetMetres);
        SmartDashboard.putBoolean(Telemetry.PREFIX + "VisionHugh/AlignedWithTarget", isAlignedWithTarget);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/TargetOffset",
            targetOffset == null ? "null" : targetOffset.toString());
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/shooterAngle", shooterAngle);
    }
}
