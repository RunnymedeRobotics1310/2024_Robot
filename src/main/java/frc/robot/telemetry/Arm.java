package frc.robot.telemetry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
    Arm() {
    }

    public double  intakeSpeed                = -1310.0;
    public double  intakeEncoderSpeed         = -1310.0;
    public double  topShooterSpeed            = -1310.0;
    public double  topShooterEncoderSpeed     = -1310.0;
    public double  bottomShooterSpeed         = -1310.0;
    public double  bottomShooterEncoderSpeed  = -1310.0;
    public double  linkPivotSpeed             = -1310.0;
    public double  linkAngle                  = -1310.0;
    public double  linkAbsoluteEncoderVoltage = -1310.0;
    public boolean isLinkAtLowerLimit         = false;
    public double  aimPivotSpeed              = -1310.0;
    public double  aimAngle                   = -1310.0;
    public double  aimAbsoluteEncoderVoltage  = -1310.0;
    public boolean noteDetected               = false;
    public boolean safetyEnabled              = false;
    public boolean trapReleased               = false;
    public Sendable trapShootTopMotorSpeedChooser    = null;
    public Sendable trapShootBottomMotorSpeedChooser = null;

    void post() {
        SmartDashboard.putBoolean(Telemetry.PREFIX + "Arm/Safety", safetyEnabled);

        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Intake/Motor", intakeSpeed);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Intake/Encoder Speed", intakeEncoderSpeed);
        SmartDashboard.putBoolean(Telemetry.PREFIX + "Arm/Intake/Note/Detected", noteDetected);

        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Shooter/TopMotor", topShooterSpeed);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Shooter/TopEncoder Speed", topShooterEncoderSpeed);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Shooter/BottomMotor", bottomShooterSpeed);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Shooter/BottomEncoder Speed", bottomShooterEncoderSpeed);
        SmartDashboard.putData(Telemetry.PREFIX + "Arm/Shooter/SetTopMotorTrapSpeed", trapShootTopMotorSpeedChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Arm/Shooter/SetBottomMotorTrapSpeed", trapShootBottomMotorSpeedChooser);

        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Link/Speed", linkPivotSpeed);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Link/Angle", linkAngle);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Link/Absolute Encoder Voltage", linkAbsoluteEncoderVoltage);
        SmartDashboard.putBoolean(Telemetry.PREFIX + "Arm/Link/Lower Limit", isLinkAtLowerLimit);

        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Aim/Speed", aimPivotSpeed);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Aim/Angle", aimAngle);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Arm/Aim/Absolute Encoder Voltage", aimAbsoluteEncoderVoltage);

        SmartDashboard.putBoolean(Telemetry.PREFIX + "Arm/Trap/Released", trapReleased);

    }
}
