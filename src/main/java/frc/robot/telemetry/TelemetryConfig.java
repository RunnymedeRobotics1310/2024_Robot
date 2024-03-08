package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelemetryConfig {

    public SendableChooser<Boolean> telemetryArmChooser        = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryClimbChooser      = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryDriveChooser      = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryHughChooser       = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryJackmanChooser    = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryLightChooser      = new SendableChooser<>();
    public SendableChooser<Boolean> telemetrySwerveChooser     = new SendableChooser<>();
    public SendableChooser<Boolean> telemetrySwerve1310Chooser = new SendableChooser<>();

    TelemetryConfig() {
        telemetryArmChooser.addOption("Enabled", true);
        telemetryArmChooser.setDefaultOption("Disabled", false);

        telemetryClimbChooser.addOption("Enabled", true);
        telemetryClimbChooser.setDefaultOption("Disabled", false);

        telemetryDriveChooser.addOption("Enabled", true);
        telemetryDriveChooser.setDefaultOption("Disabled", false);

        telemetryHughChooser.addOption("Enabled", true);
        telemetryHughChooser.setDefaultOption("Disabled", false);

        telemetryJackmanChooser.addOption("Enabled", true);
        telemetryJackmanChooser.setDefaultOption("Disabled", false);

        telemetryLightChooser.addOption("Enabled", true);
        telemetryLightChooser.setDefaultOption("Disabled", false);

        telemetrySwerveChooser.addOption("Enabled", true);
        telemetrySwerveChooser.setDefaultOption("Disabled", false);

        telemetrySwerve1310Chooser.addOption("Enabled", true);
        telemetrySwerve1310Chooser.setDefaultOption("Disabled", false);
    }

    void post() {
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Arm", telemetryArmChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Climb", telemetryClimbChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Drive", telemetryDriveChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Hugh", telemetryHughChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Jackman", telemetryJackmanChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Light", telemetryLightChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Swerve", telemetrySwerveChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Swerve1310", telemetrySwerve1310Chooser);
    }

    boolean arm() {
        Boolean enabled = telemetryArmChooser.getSelected();
        return enabled != null && enabled;
    }

    boolean climb() {
        Boolean enabled = telemetryClimbChooser.getSelected();
        return enabled != null && enabled;
    }

    boolean drive() {
        Boolean enabled = telemetryDriveChooser.getSelected();
        return enabled != null && enabled;
    }

    boolean hugh() {
        Boolean enabled = telemetryHughChooser.getSelected();
        return enabled != null && enabled;
    }

    boolean jackman() {
        Boolean enabled = telemetryJackmanChooser.getSelected();
        return enabled != null && enabled;
    }

    boolean light() {
        Boolean enabled = telemetryLightChooser.getSelected();
        return enabled != null && enabled;
    }

    boolean swerve() {
        Boolean enabled = telemetrySwerveChooser.getSelected();
        return enabled != null && enabled;
    }

    boolean swerve1310() {
        Boolean enabled = telemetrySwerve1310Chooser.getSelected();
        return enabled != null && enabled;
    }

}