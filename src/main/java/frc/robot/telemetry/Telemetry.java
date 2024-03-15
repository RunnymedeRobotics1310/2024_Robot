package frc.robot.telemetry;

public class Telemetry {

    public static final String    PREFIX = "1310/";

    public static TelemetryConfig config = new TelemetryConfig();

    public static Arm             arm    = new Arm();
    public static Climb           climb  = new Climb();
    public static Test            test   = new Test();

    private Telemetry() {
    }

    public static void post() {

        config.post();

        if (config.arm()) {
            arm.post();
        }
        if (config.climb()) {
            climb.post();
        }
        test.post();
    }
}