package frc.robot.telemetry;

public class Telemetry {

    public static final String    PREFIX  = "1310/";

    public static TelemetryConfig config  = new TelemetryConfig();

    public static Arm             arm     = new Arm();
    public static Drive           drive   = new Drive();
    public static Light           light   = new Light();
    public static Swerve          swerve  = new Swerve();

    private Telemetry() {
    }

    public static void post() {

        config.post();

        if (config.arm()) {
            arm.post();
        }
        if (config.drive()) {
            drive.post();
        }
        if (config.light()) {
            light.post();
        }
        if (config.swerve()) {
            swerve.post();
        }
    }
}