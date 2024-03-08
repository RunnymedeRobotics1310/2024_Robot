package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb {
    Climb() {
    }

    public double  leftClimbSpeed    = -1310.0;
    public double  leftClimbEncoder  = -1310.0;
    public double  rightClimbSpeed   = -1310.0;
    public double  rightClimbEncoder = -1310.0;
    public boolean rightLimit        = false;
    public boolean leftLimit         = false;

    void post() {
        SmartDashboard.putNumber(Telemetry.PREFIX + "Climb/Left Speed", leftClimbSpeed);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Climb/Left Encoder", leftClimbEncoder);

        SmartDashboard.putNumber(Telemetry.PREFIX + "Climb/Right Speed", rightClimbSpeed);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Climb/Right Encoder", rightClimbEncoder);

        SmartDashboard.putBoolean(Telemetry.PREFIX + "Climb/Right Limit", rightLimit);
        SmartDashboard.putBoolean(Telemetry.PREFIX + "Climb/Left Limit", leftLimit);
    }
}
