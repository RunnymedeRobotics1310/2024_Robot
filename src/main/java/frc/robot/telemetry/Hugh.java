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

    void post() {
        SmartDashboard.putString("VisionHugh/BotTarget", botTarget.toString());
        SmartDashboard.putString("VisionHugh/PriorityId", "" + priorityId);
        SmartDashboard.putBoolean("VisionHugh/Target Found", targetFound);
        SmartDashboard.putNumber("VisionHugh/tid", tid);
        SmartDashboard.putNumber("VisionHugh/tx", tx);
        SmartDashboard.putNumber("VisionHugh/ty", ty);
        SmartDashboard.putNumber("VisionHugh/ta", ta);
        SmartDashboard.putNumber("VisionHugh/tl", tl);
        SmartDashboard.putString("VisionHugh/Botpose", Arrays.toString(botpost));
        SmartDashboard.putNumber("VisionHugh/TargetAvgDist", targetAvgDist);
        SmartDashboard.putString("VisionHugh/PoseConf",
            visPose == null ? PoseConfidence.NONE.toString() : visPose.poseConfidence().toString());
        SmartDashboard.putString("VisionHugh/NumTags", "" + numTags);
        SmartDashboard.putString("VisionHugh/AprilTagInfo", aprilTagInfo);
        SmartDashboard.putNumber("VisionHugh/DistToTarget", distanceToTargetMetres);
        SmartDashboard.putBoolean("VisionHugh/AlignedWithTarget", isAlignedWithTarget);
        SmartDashboard.putString("VisionHugh/TargetOffset", targetOffset == null ? "null" : targetOffset.toString());
    }
}
