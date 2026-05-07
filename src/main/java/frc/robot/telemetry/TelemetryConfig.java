package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelemetryConfig {

    public SendableChooser<Boolean> telemetryArmChooser        = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryDriveChooser      = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryLightChooser      = new SendableChooser<>();
    public SendableChooser<Boolean> telemetrySwerveChooser     = new SendableChooser<>();
    public SendableChooser<Boolean> telemetrySwerve1310Chooser = new SendableChooser<>();

    TelemetryConfig() {
        telemetryArmChooser.addOption("Enabled", true);
        telemetryArmChooser.setDefaultOption("Disabled", false);

        telemetryDriveChooser.setDefaultOption("Enabled", true);
        telemetryDriveChooser.addOption("Disabled", false);

        telemetryLightChooser.addOption("Enabled", true);
        telemetryLightChooser.setDefaultOption("Disabled", false);

        telemetrySwerveChooser.addOption("Enabled", true);
        telemetrySwerveChooser.setDefaultOption("Disabled", false);

        telemetrySwerve1310Chooser.addOption("Enabled", true);
        telemetrySwerve1310Chooser.setDefaultOption("Disabled", false);
    }

    void post() {
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Arm", telemetryArmChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Drive", telemetryDriveChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Light", telemetryLightChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Swerve", telemetrySwerveChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Swerve1310", telemetrySwerve1310Chooser);
    }

    public boolean arm() {
        Boolean enabled = telemetryArmChooser.getSelected();
        return enabled != null && enabled;
    }

    public boolean drive() {
        Boolean enabled = telemetryDriveChooser.getSelected();
        return enabled != null && enabled;
    }

    public boolean light() {
        Boolean enabled = telemetryLightChooser.getSelected();
        return enabled != null && enabled;
    }

    public boolean swerve() {
        Boolean enabled = telemetrySwerveChooser.getSelected();
        return enabled != null && enabled;
    }

    public boolean swerve1310() {
        Boolean enabled = telemetrySwerve1310Chooser.getSelected();
        return enabled != null && enabled;
    }

}
