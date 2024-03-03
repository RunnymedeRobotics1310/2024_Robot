package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.NOTE_ORANGE;
import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * Light signal to display while the robot is actively in intake mode.
 *
 * If performance permits, this will flash the orange note colour.
 *
 * If this is not possible due to heavy demands on performance, this will render a pattern in which
 * every third light is orange and the rest are off.
 *
 * TODO: Check to see if we can flash the orange note colour - watch loop overruns.
 */
public class Intaking extends LightingPattern {

    private static final LightingPattern INSTANCE = new Intaking();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private Intaking() {
        super(SIGNAL);
        for (int i = 0; i < buffer.getLength(); i++) {
            if (i % 3 == 0) {
                buffer.setLED(i, NOTE_ORANGE);
            }
            else {
                buffer.setLED(i, Color.kBlack);
            }
        }
    }
}
