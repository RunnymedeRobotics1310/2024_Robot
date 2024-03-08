package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
    Arm() {
    }

    public double  intakeSpeed                = -1310.0;
    public double  intakeEncoderSpeed         = -1310.0;
    public double  shooterSpeed               = -1310.0;
    public double  shooterEncoderSpeed        = -1310.0;
    public double  linkPivotSpeed             = -1310.0;
    public double  linkAngle                  = -1310.0;
    public double  linkAbsoluteEncoderVoltage = -1310.0;
    public boolean isLinkAtLowerLimit         = false;
    public double  aimPivotSpeed              = -1310.0;
    public double  aimAngle                   = -1310.0;
    public double  aimAbsoluteEncoderVoltage  = -1310.0;
    public boolean noteDetected               = false;
    public boolean safetyEnabled              = false;

    void post() {
        SmartDashboard.putNumber("Intake Motor", intakeSpeed);
        SmartDashboard.putNumber("Intake Encoder Speed", intakeEncoderSpeed);

        SmartDashboard.putNumber("Shooter Motor", shooterSpeed);
        SmartDashboard.putNumber("Shooter Encoder Speed", shooterEncoderSpeed);

        SmartDashboard.putNumber("Link Speed", linkPivotSpeed);
        SmartDashboard.putNumber("Link Angle", linkAngle);
        SmartDashboard.putNumber("Link Absolute Encoder Voltage", linkAbsoluteEncoderVoltage);
        SmartDashboard.putBoolean("Link Lower Limit", isLinkAtLowerLimit);

        SmartDashboard.putNumber("Aim Speed", aimPivotSpeed);
        SmartDashboard.putNumber("Aim Angle", aimAngle);
        SmartDashboard.putNumber("Aim Absolute Encoder Voltage", aimAbsoluteEncoderVoltage);

        SmartDashboard.putBoolean("Note Detected", noteDetected);

        SmartDashboard.putBoolean("Arm Safety", safetyEnabled);
    }
}
