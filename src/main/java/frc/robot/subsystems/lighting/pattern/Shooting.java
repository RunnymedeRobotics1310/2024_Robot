package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * shooting -> rainbow
 */
public class Shooting extends LightingPattern {
    public Shooting() {
        super(SIGNAL);
    }

    @Override
    public AddressableLEDBuffer periodic() {
        AddressableLEDBuffer buffer = SIGNAL.createBuffer();
        clearLEDs(buffer);
        return buffer;
    }

}
