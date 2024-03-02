package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import static frc.robot.Constants.LightingConstants.SIGNAL;

/**
 * Test mode active
 */
public class TestMode extends LightingPattern {
    public TestMode() {
        super(SIGNAL);
    }

    @Override
    public AddressableLEDBuffer periodic() {
        AddressableLEDBuffer buffer = SIGNAL.createBuffer();
        clearLEDs(buffer);
        return buffer;
    }

}
