package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public abstract class LightingPattern {
    public final int                     length;
    protected final AddressableLEDBuffer buffer;

    public LightingPattern(int length) {
        this.length = length;
        this.buffer = new AddressableLEDBuffer(length);
    }

    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }
}