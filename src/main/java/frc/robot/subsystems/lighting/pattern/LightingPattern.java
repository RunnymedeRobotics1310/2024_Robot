package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.lighting.LightStripRegion;

public abstract class LightingPattern {
    protected final LightStripRegion region;

    public LightingPattern(LightStripRegion region) {
        this.region = region;
    }

    public LightStripRegion getRegion() {
        return region;
    }

    public abstract AddressableLEDBuffer periodic();

    /**
     * Clear the entire LED strip
     */
    protected final void clearLEDs(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }
}
