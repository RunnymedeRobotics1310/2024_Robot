package frc.robot.subsystems.lighting.pattern;

import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * Light signal to display while the robot is doing nothing in particular.
 *
 * If performance permits, this will pulse the current alliance colour.
 *
 * If this is not possible due to heavy demands on performance, all lights will be off.
 *
 * // TODO: Determine if pulsing the alliance colour is possible - watch loop overruns.
 */
public class Default extends LightingPattern {

    private static final LightingPattern INSTANCE = new Default();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private Default() {
        super(SIGNAL);
        clearLEDs();
    }
}
