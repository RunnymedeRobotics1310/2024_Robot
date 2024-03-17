package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.VISPOSE2;

/**
 * Light signal to display when robot vision pose measurements have a medium confidence.
 *
 * This pattern displays the lights in a solid cyan pattern.
 *
 * @see VisionConfidenceHigh
 * @see VisionConfidenceLow
 * @see VisionConfidenceNone
 */
public class VisionConfidenceMedium2 extends LightingPattern {

    private static final LightingPattern INSTANCE = new VisionConfidenceMedium2();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private VisionConfidenceMedium2() {
        super(VISPOSE2);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kCyan);
        }
    }
}