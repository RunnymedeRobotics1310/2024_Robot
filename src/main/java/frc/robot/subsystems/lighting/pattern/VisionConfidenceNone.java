package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.VISPOSE1;

/**
 * Light signal to display when robot vision pose measurements have a no confidence.
 *
 * This pattern lights are all off in the region.
 *
 * @see VisionConfidenceHigh
 * @see VisionConfidenceMedium
 * @see VisionConfidenceLow
 */
public class VisionConfidenceNone extends LightingPattern {

    private static final LightingPattern INSTANCE = new VisionConfidenceNone();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private VisionConfidenceNone() {
        super(VISPOSE1.length);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kViolet);
        }
    }
}
