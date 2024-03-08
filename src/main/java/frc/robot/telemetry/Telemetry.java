package frc.robot.telemetry;

public class Telemetry {

    public static Arm        arm        = new Arm();
    public static Auto       auto       = new Auto();
    public static Climb      climb      = new Climb();
    public static Drive      drive      = new Drive();
    public static Hugh       hugh       = new Hugh();
    public static Jackman    jackman    = new Jackman();
    public static Light      light      = new Light();
    public static SwerveCore swerve     = new SwerveCore();
    public static SwervePlus swerve1310 = new SwervePlus();
    public static Test       test       = new Test();

    private Telemetry() {
    }

    public static void post() {

        arm.post();
        auto.post();
        climb.post();
        drive.post();
        hugh.post();
        jackman.post();
        light.post();
        swerve.post();
        swerve1310.post();
        test.post();
    }
}