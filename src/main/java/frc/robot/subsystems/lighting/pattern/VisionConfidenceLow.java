package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.VISPOSE;

/**
 * Light signal to display when robot vision pose measurements have a low confidence.
 *
 * This pattern displays the lights in a solid purple pattern.
 *
 * @see VisionConfidenceHigh
 * @see VisionConfidenceMedium
 * @see VisionConfidenceNone
 */
public class VisionConfidenceLow extends LightingPattern {

    private static final LightingPattern INSTANCE = new VisionConfidenceLow();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private VisionConfidenceLow() {
        super(VISPOSE);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kPurple);
        }
    }
}