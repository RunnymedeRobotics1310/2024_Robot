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



    void post() {
        SmartDashboard.putBoolean("LimelightJackman/Target Found", isVisionTargetFound);
        SmartDashboard.putNumber("LimelightJackman/tx-value", tx);
        SmartDashboard.putNumber("LimelightJackman/ty-value", ty);
        SmartDashboard.putNumber("LimelightJackman/ta-value", ta);
        SmartDashboard.putNumber("LimelightJackman/l-value", tl);
        SmartDashboard.putNumber("LimelightJackman/Cam Mode", camMode);
        SmartDashboard.putNumber("LimelightJackman/LED mode", ledMode);
        SmartDashboard.putNumber("LimelightJackman/Pipeline", pipeline);
    }
}
