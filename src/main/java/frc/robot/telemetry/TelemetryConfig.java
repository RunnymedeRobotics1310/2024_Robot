package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelemetryConfig {

    public SendableChooser<Boolean> telemetryArmChooser        = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryAutoChooser       = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryClimbChooser      = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryDriveChooser      = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryHughChooser       = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryJackmanChooser    = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryLightChooser      = new SendableChooser<>();
    public SendableChooser<Boolean> telemetrySwerveChooser     = new SendableChooser<>();
    public SendableChooser<Boolean> telemetrySwerve1310Chooser = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryTestChooser       = new SendableChooser<>();

    TelemetryConfig() {
        telemetryArmChooser.addOption("Enabled", true);
        telemetryArmChooser.setDefaultOption("Disabled", false);

        telemetryAutoChooser.addOption("Enabled", true);
        telemetryAutoChooser.setDefaultOption("Disabled", false);

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

        telemetrySwerveChooser.setDefaultOption("Enabled", true);
        telemetrySwerveChooser.addOption("Disabled", false);

        telemetrySwerve1310Chooser.addOption("Enabled", true);
        telemetrySwerve1310Chooser.setDefaultOption("Disabled", false);

        telemetryTestChooser.setDefaultOption("Enabled", true);
        telemetryTestChooser.addOption("Disabled", false);
    }

    void post() {
        SmartDashboard.putData("Arm Telemetry", telemetryArmChooser);
        SmartDashboard.putData("Auto Telemetry", telemetryAutoChooser);
        SmartDashboard.putData("Climb Telemetry", telemetryClimbChooser);
        SmartDashboard.putData("Drive Telemetry", telemetryDriveChooser);
        SmartDashboard.putData("Hugh Telemetry", telemetryHughChooser);
        SmartDashboard.putData("Jackman Telemetry", telemetryJackmanChooser);
        SmartDashboard.putData("Light Telemetry", telemetryLightChooser);
        SmartDashboard.putData("Swerve Telemetry", telemetrySwerveChooser);
        SmartDashboard.putData("Swerve1310 Telemetry", telemetrySwerve1310Chooser);
        SmartDashboard.putData("Test Telemetry", telemetryTestChooser);
    }

    boolean arm() {
        Boolean enabled = telemetryArmChooser.getSelected();
        return enabled != null && enabled;
    }

    boolean auto() {
        Boolean enabled = telemetryAutoChooser.getSelected();
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

    boolean test() {
        Boolean enabled = telemetryTestChooser.getSelected();
        return enabled != null && enabled;
    }

}
