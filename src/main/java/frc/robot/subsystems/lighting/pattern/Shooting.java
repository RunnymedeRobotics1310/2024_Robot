package frc.robot.subsystems.lighting.pattern;

import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * Light signal to display while the robot is actively shooting.
 *
 * If performance permits, this will render a scrolling rainbow pattern.
 *
 * If this is not possible due to heavy demands on performance, this will render a fixed rainbow.
 *
 * TODO: Check to see if we can render a scrolling rainbow pattern - watch loop overruns.
 * 
 * @see "https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html"
 */
public class Shooting extends LightingPattern {

    private static final LightingPattern INSTANCE = new Shooting();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private Shooting() {
        super(SIGNAL);
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (i * 180 / buffer.getLength()) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
    }
}
