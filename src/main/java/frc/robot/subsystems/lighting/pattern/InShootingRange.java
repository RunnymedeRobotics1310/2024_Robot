package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * within shooting range -> orange pulse
 */
public class InShootingRange extends LightingPattern {
    public InShootingRange() {
        super(SIGNAL);
    }

    @Override
    public AddressableLEDBuffer periodic() {
        AddressableLEDBuffer buffer = SIGNAL.createBuffer();
        clearLEDs(buffer);
        return buffer;
    }

}
