package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.lighting.LightstripRegion;

public abstract class LightingPattern {
    protected final LightstripRegion     region;
    protected final AddressableLEDBuffer buffer;

    public LightingPattern(LightstripRegion region) {
        this.region = region;
        this.buffer = region.createBuffer();
    }

    public LightstripRegion getRegion() {
        return region;
    }

    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }
}