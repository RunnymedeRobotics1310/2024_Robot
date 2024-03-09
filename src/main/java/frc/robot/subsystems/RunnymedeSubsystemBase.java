package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RunnymedeSubsystemBase extends SubsystemBase {
    public RunnymedeSubsystemBase() {
    }

    public RunnymedeSubsystemBase(String name) {
        super(name);
    }

    protected void log(String msg) {
        System.out.println(msg);
    }
}
