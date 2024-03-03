package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * Test mode active - alternating red and white lights
 */
public class TestMode extends LightingPattern {

    public static final LightingPattern INSTANCE = new TestMode();

    private final AddressableLEDBuffer  buffer   = SIGNAL.createBuffer();

    private TestMode() {
        super(SIGNAL);
        for (int i = 0; i < buffer.getLength(); i++) {
            if (i % 2 == 0) {
                buffer.setLED(i, Color.kRed);
            }
            else {
                buffer.setLED(i, Color.kWhite);
            }
        }
    }

    @Override
    public AddressableLEDBuffer periodic() {
        return buffer;
    }

}
