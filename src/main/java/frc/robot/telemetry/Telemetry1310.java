package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.color.CMMException;

public class Telemetry1310 {

    public static Drive   drive   = new Drive();
    public static Auto    auto    = new Auto();
    public static Hugh    hugh    = new Hugh();
    public static Jackman jackman = new Jackman();
    public static Test    test    = new Test();
    public static Light   light   = new Light();
    public static Arm     arm     = new Arm();
    public static Climb   climb   = new Climb();

    private Telemetry1310() {
    }

    public static String allianceName = null;

    public static void post() {

        SmartDashboard.putString("Drive/Teleop/Alliance", allianceName);

        drive.post();
        auto.post();
        hugh.post();
        jackman.post();
        test.post();
        light.post();
        arm.post();
        climb.post();
    }

}


