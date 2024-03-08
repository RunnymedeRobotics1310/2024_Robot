package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb {
    Climb() {
    }

    public double  leftClimbSpeed    = -1310.0;
    public double  leftClimbEncoder  = -1310.0;
    public double  rightClimbSpeed   = -1310.0;
    public double  rightClimbEncoder = -1310.0;
    public boolean safetyEnabled     = false;
    public boolean isAtLimit2        = false;
    public boolean isAtLimit3        = false;

    void post() {
        SmartDashboard.putNumber("Left Climb Speed", leftClimbSpeed);
        SmartDashboard.putNumber("Left Climb Encoder", leftClimbEncoder);

        SmartDashboard.putNumber("Right Climb Speed", rightClimbSpeed);
        SmartDashboard.putNumber("Right Climb Encoder", rightClimbEncoder);

        SmartDashboard.putBoolean("Climb Safety", safetyEnabled);

        SmartDashboard.putBoolean("Climb Limit 2", isAtLimit2);
        SmartDashboard.putBoolean("Climb limit 3", isAtLimit3);
    }
}
