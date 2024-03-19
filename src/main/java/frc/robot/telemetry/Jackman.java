package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Jackman {
    Jackman() {
    }

    public boolean isVisionTargetFound = false;
    public double  tx                  = -1310.0;
    public double  ty                  = -1310.0;
    public double  ta                  = -1310.0;
    public double  tl                  = -1310.0;
    public double  camMode             = -1310.0;
    public double  ledMode             = -1310.0;
    public double  pipeline            = -1310.0;
    public double  noteOffset          = -1310.0;

    void post() {
        SmartDashboard.putBoolean(Telemetry.PREFIX + "VisionJackman/Target Found", isVisionTargetFound);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionJackman/tx-value", tx);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionJackman/ty-value", ty);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionJackman/ta-value", ta);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionJackman/l-value", tl);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionJackman/Cam Mode", camMode);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionJackman/LED mode", ledMode);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionJackman/Pipeline", pipeline);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionJackman/NoteOffset", noteOffset);
    }
}
