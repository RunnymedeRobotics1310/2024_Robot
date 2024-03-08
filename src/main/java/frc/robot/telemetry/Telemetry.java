package frc.robot.telemetry;

public class Telemetry {

    public static final String    PREFIX  = "1310/";

    public static TelemetryConfig config  = new TelemetryConfig();

    public static Arm             arm     = new Arm();
    public static Auto            auto    = new Auto();
    public static Climb           climb   = new Climb();
    public static Drive           drive   = new Drive();
    public static Hugh            hugh    = new Hugh();
    public static Jackman         jackman = new Jackman();
    public static Light           light   = new Light();
    public static SwerveCore      swerve  = new SwerveCore();
    public static Test            test    = new Test();

    private Telemetry() {
    }

    public static void post() {

        config.post();

        if (config.arm()) {
            arm.post();
        }
        auto.post();
        if (config.climb()) {
            climb.post();
        }
        if (config.drive()) {
            drive.post();
        }
        if (config.hugh()) {
            hugh.post();
        }
        if (config.jackman()) {
            jackman.post();
        }
        if (config.light()) {
            light.post();
        }
        if (config.swerve()) {
            swerve.post();
        }
        test.post();
    }
}