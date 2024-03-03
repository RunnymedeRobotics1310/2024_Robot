package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * Light signal to display while the robot is in test mode.
 *
 * This pattern displays the lights in ana alternating red and white pattern, without motion.
 */
public class TestMode extends LightingPattern {

    private static final LightingPattern INSTANCE = new TestMode();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

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

}
