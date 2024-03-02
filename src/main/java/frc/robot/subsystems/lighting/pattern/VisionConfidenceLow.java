package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.VISPOSE;

/**
 * vision confidence -> green, blue, purple
 */
public class VisionConfidenceLow extends LightingPattern {
    public static LightingPattern      INSTANCE = new VisionConfidenceLow();

    private final AddressableLEDBuffer buffer;

    private VisionConfidenceLow() {
        super(VISPOSE);
        buffer = VISPOSE.createBuffer();
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kPurple);
        }
    }

    @Override
    public AddressableLEDBuffer periodic() {
        return buffer;
    }
}