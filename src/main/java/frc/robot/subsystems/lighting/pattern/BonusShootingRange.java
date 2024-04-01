package frc.robot.subsystems.lighting.pattern;


import static frc.robot.Constants.LightingConstants.NOTE_RED;
import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * Light signal to display when the robot is within shooting range of a target.
 *
 * If performance permits, this will pulse the orange note colour.
 *
 * If this is not possible due to heavy demands on performance, a solid orange pattern will be
 * shown instead.
 *
 * TODO: Check to see if we can pulse the orange note colour - watch loop overruns.
 */
public class BonusShootingRange extends LightingPattern {

    private static final LightingPattern INSTANCE = new BonusShootingRange();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }


    private BonusShootingRange() {
        super(SIGNAL.length);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, NOTE_RED);
        }
    }
}
