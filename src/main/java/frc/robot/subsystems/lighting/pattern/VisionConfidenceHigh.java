package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.VISPOSE;

/**
 * Light signal to display when robot vision pose measurements have a high confidence.
 *
 * This pattern displays the lights in a solid green pattern.
 *
 * @see VisionConfidenceMedium
 * @see VisionConfidenceLow
 * @see VisionConfidenceNone
 */
public class VisionConfidenceHigh extends LightingPattern {

    private static final LightingPattern INSTANCE = new VisionConfidenceHigh();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private VisionConfidenceHigh() {
        super(VISPOSE);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kGreen);
        }
    }
}
