package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * Light signal to display while the robot is climbing.
 *
 * If performance permits, this will flash the current alliance colour.
 *
 * If this is not possible due to heavy demands on performance, a solid green pattern will be
 * displayed.
 *
 * TODO: Check to see if we can flash the alliance colour - watch loop overruns.
 */
public class Climbing extends LightingPattern {

    private static final LightingPattern INSTANCE = new Climbing();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private Climbing() {
        super(SIGNAL.length);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kGreen);
        }
    }

}
