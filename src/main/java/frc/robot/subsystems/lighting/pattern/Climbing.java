package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * climb in progress -> alliance flash
 */
public class Climbing extends LightingPattern {
    public Climbing() {
        super(SIGNAL);
    }

    @Override
    public AddressableLEDBuffer periodic() {
        AddressableLEDBuffer buffer = SIGNAL.createBuffer();
        clearLEDs(buffer);
        return buffer;
    }
}
