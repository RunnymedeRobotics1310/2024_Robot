package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * default -> alliance pulse
 */
public class Default extends LightingPattern {

    private static final LightingPattern INSTANCE = new Default();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private Default() {
        super(SIGNAL);
    }

    @Override
    public AddressableLEDBuffer periodic() {
        AddressableLEDBuffer buffer = SIGNAL.createBuffer();
        clearLEDs(buffer);
        return buffer;
    }

}
