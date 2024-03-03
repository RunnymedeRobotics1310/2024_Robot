package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

/**
 * Light signal to display while the robot is doing nothing in particular.
 *
 * If performance permits, this will pulse the current alliance colour.
 *
 * If this is not possible due to heavy demands on performance, all lights will be the alliance
 * color.
 *
 * // TODO: Determine if pulsing the alliance colour is possible - watch loop overruns.
 */
public class Default extends LightingPattern {

    private static final AddressableLEDBuffer RED_BUFFER  = SIGNAL.createBuffer();
    private static final AddressableLEDBuffer BLUE_BUFFER = SIGNAL.createBuffer();
    static {
        for (int i = 0; i < RED_BUFFER.getLength(); i++) {
            RED_BUFFER.setLED(i, Color.kFirstRed);
            BLUE_BUFFER.setLED(i, Color.kFirstBlue);
        }
    }

    private static final LightingPattern INSTANCE = new Default();


    public static LightingPattern getInstance() {
        return INSTANCE;
    }

    private Default() {
        super(SIGNAL);
    }

    @Override
    public AddressableLEDBuffer periodic() {
        if (getRunnymedeAlliance() == Red) {
            return RED_BUFFER;
        }
        else {
            return BLUE_BUFFER;
        }
    }
}
