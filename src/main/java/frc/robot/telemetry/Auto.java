package frc.robot.telemetry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.telemetry.Telemetry.PREFIX;

public class Auto {
    Auto() {
    }

    public Sendable autoPatternChooser = null;
    public Sendable delayChooser = null;

    public void post() {
        SmartDashboard.putData(PREFIX + "Auto Pattern", autoPatternChooser);
        SmartDashboard.putData(PREFIX + "Delay", delayChooser);
    }
}