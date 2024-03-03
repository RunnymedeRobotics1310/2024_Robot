package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
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

    public AddressableLEDBuffer periodic() {
        return buffer;
    }

    /**
     * Clear the entire LED strip
     */
    protected final void clearLEDs() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }
}
