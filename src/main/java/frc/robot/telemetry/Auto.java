package frc.robot.telemetry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto {
    Auto() {
    }

    public Sendable autoPatternChooser = null;

    public void post() {
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
    }
}