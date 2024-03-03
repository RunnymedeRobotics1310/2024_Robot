package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * within shooting range -> orange pulse
 */
public class InShootingRange extends LightingPattern {

    public static LightingPattern getInstance() {
        return new InShootingRange();
    }

    // todo: add pulse effect
    private static final Color         NOTE_COLOR = new Color(255, 20, 0);

    private final AddressableLEDBuffer buffer     = SIGNAL.createBuffer();

    private InShootingRange() {
        super(SIGNAL);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, NOTE_COLOR);
        }
    }

    @Override
    public AddressableLEDBuffer periodic() {
        return buffer;
    }

}
