package frc.robot.subsystems.lighting.pattern;


import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.Constants.LightingConstants.NOTE_ORANGE;

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
public class InShootingRange extends LightingPattern {

    private static final LightingPattern INSTANCE = new InShootingRange();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }


    private InShootingRange() {
        super(SIGNAL);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, NOTE_ORANGE);
        }
    }
}
