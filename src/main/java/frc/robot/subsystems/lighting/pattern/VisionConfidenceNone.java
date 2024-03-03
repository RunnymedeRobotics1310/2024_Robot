package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.VISPOSE;

public class VisionConfidenceNone extends LightingPattern {

    private static final LightingPattern INSTANCE = new VisionConfidenceNone();

    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private final AddressableLEDBuffer buffer;

    private VisionConfidenceNone() {
        super(VISPOSE);
        buffer = VISPOSE.createBuffer();
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }

    @Override
    public AddressableLEDBuffer periodic() {
        return buffer;
    }
}
