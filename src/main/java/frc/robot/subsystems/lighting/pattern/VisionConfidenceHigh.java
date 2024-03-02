package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.VISPOSE;

/**
 * vision confidence -> green, blue, purple
 */
public class VisionConfidenceHigh extends LightingPattern {

    public static LightingPattern      INSTANCE = new VisionConfidenceHigh();

    private final AddressableLEDBuffer buffer;

    private VisionConfidenceHigh() {
        super(VISPOSE);
        buffer = VISPOSE.createBuffer();
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kGreen);
        }
    }

    @Override
    public AddressableLEDBuffer periodic() {
        return buffer;
    }
}
