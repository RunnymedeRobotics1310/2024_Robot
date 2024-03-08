package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.LinkedHashMap;

public class Light {
    Light() {
    }

    public String                              lightstripRegionWarning = null;

    public final LinkedHashMap<String, String> regionStatus            = new LinkedHashMap<>();

    void post() {
        SmartDashboard.putString(Telemetry.PREFIX + "Lighting/Warning",
            lightstripRegionWarning == null ? "" : lightstripRegionWarning);

        regionStatus.forEach((k, v) -> {
            if (v != null) {
                SmartDashboard.putString(Telemetry.PREFIX + "Lighting/" + k, v);
            }
        });


    }
}
