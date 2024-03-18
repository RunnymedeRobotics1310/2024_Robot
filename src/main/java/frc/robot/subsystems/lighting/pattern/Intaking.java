package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LightingConstants.NOTE_ORANGE;
import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

/**
 * Light signal to display while the robot is actively in intake mode.
 *
 * If performance permits, this will flash the orange note colour.
 *
 * If this is not possible due to heavy demands on performance, this will render a pattern in which
 * every third light is orange and the rest are off.
 *
 */
public class Intaking extends LightingPattern {

    private static final LightingPattern RED  = new Intaking(NOTE_ORANGE, Color.kRed);
    private static final LightingPattern BLUE = new Intaking(NOTE_ORANGE, Color.kFirstBlue);

    public static LightingPattern getInstance() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            return RED;
        }
        else {
            return BLUE;
        }
    }

    private final AddressableLEDBuffer offBuffer;

    private Intaking(Color on, Color off) {
        super(SIGNAL.length);
        offBuffer = new AddressableLEDBuffer(SIGNAL.length);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, on);
            offBuffer.setLED(i, off);
        }
    }

    public AddressableLEDBuffer getBuffer() {
        if (RobotController.getRSLState()) {
            return buffer;
        }
        else {
            return offBuffer;
        }
    }
}
