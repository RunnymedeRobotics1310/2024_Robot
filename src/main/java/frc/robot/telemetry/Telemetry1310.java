package frc.robot.telemetry;

public class Telemetry1310 {

    public static Arm        arm        = new Arm();
    public static Auto       auto       = new Auto();
    public static Climb      climb      = new Climb();
    public static Drive      drive      = new Drive();
    public static Hugh       hugh       = new Hugh();
    public static Jackman    jackman    = new Jackman();
    public static Light      light      = new Light();
    public static SwerveCore swerveCore = new SwerveCore();
    public static SwervePlus swervePlus = new SwervePlus();
    public static Test       test       = new Test();

    private Telemetry1310() {
    }

    public static void post() {

        arm.post();
        auto.post();
        climb.post();
        drive.post();
        hugh.post();
        jackman.post();
        light.post();
        swerveCore.post();
        swervePlus.post();
        test.post();
    }
}