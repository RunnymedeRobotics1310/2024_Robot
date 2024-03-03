package frc.robot.subsystems.lighting;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.lighting.pattern.Default;
import frc.robot.subsystems.lighting.pattern.LightingPattern;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceNone;

public class LightingSubsystem extends SubsystemBase {


    /*
     * Lighting Signals
     * - Intake in progress -> orange flash
     * - within shooting range -> orange pulse
     * - climb in progress -> alliance flash
     * - default -> alliance pulse
     * - shooting -> rainbow
     * - vision confidence -> green, blue, purple
     * - test mode -> purple
     */


    private final AddressableLED       ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    private LightingPattern            visionPattern = VisionConfidenceNone.INSTANCE;

    private LightingPattern            signalPattern = new Default();



    public LightingSubsystem() {

        ledStrip  = new AddressableLED(Constants.LightingConstants.LIGHT_STRING_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LightingConstants.LIGHT_STRING_LENGTH);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);

        ledStrip.start();
    }

    public void setVisionPattern(LightingPattern pattern) {
        if (pattern == null) {
            pattern = VisionConfidenceNone.INSTANCE;
        }
        visionPattern = pattern;
    }

    public void setSignalPattern(LightingPattern pattern) {
        if (pattern == null) {
            pattern = new Default();
        }
        signalPattern = pattern;
    }

    @Override
    public void periodic() {

        // apply vision pattern
        AddressableLEDBuffer visionBuffer = visionPattern.periodic();
        for (int i = 0; i < visionBuffer.getLength(); i++) {
            int idx = visionPattern.getRegion().start + i;
            if (idx < ledBuffer.getLength()) {
                ledBuffer.setLED(idx, visionBuffer.getLED(i));
            }
        }

        // apply signal pattern
        AddressableLEDBuffer signalBuffer = signalPattern.periodic();
        for (int i = 0; i < signalBuffer.getLength(); i++) {
            int idx = signalPattern.getRegion().start + i;
            if (idx < ledBuffer.getLength()) {
                ledBuffer.setLED(idx, signalBuffer.getLED(i));
            }
        }

        safelySetLights();
    }

    private void safelySetLights() {

        /*
         * To stay within the RIO bounds, we would need to
         * draw < 2A or 12W total power for the LEDs the way
         * they are configured.
         *
         * The lights probably draw about 20mA/colour
         *
         * @ full on, so RED=20mA, BLUE=20mA, WHITE=60mA.
         * Less if the LEDs are dimly lit. Single colour,
         * we can safely light 100 LEDs, Full white, about 33 LEDs.
         */
        double               MAX_MILLIAMPS = 10000;

        // don't destroy the desired levels by overwriting buffer
        AddressableLEDBuffer dimmedBuffer  = null;

        double               ratio;
        do {
            AddressableLEDBuffer workingBuffer = dimmedBuffer == null ? ledBuffer : dimmedBuffer;
            double               milliamps     = 0;
            for (int i = 0; i < workingBuffer.getLength(); i++) {
                Color color = workingBuffer.getLED(i);
                milliamps += color.red * 20;
                milliamps += color.blue * 20;
                milliamps += color.green * 20;
            }
            ratio = milliamps / MAX_MILLIAMPS;
            if (ratio >= 1) {
                if (dimmedBuffer == null) {
                    dimmedBuffer = new AddressableLEDBuffer(ledBuffer.getLength());
                }
                for (int i = 0; i < workingBuffer.getLength(); i++) {
                    Color  color       = workingBuffer.getLED(i);
                    double r           = color.red / ratio;
                    double g           = color.green / ratio;
                    double b           = color.blue / ratio;
                    Color  dimmerColor = new Color(Math.max(0, r), Math.max(0, g), Math.max(0, b));
                    dimmedBuffer.setLED(i, dimmerColor);
                }
            }

        }
        while (ratio > 1);


        if (dimmedBuffer != null) {
            System.out.println("Dimming lights to reduce current draw");
            ledStrip.setData(dimmedBuffer);
        }
        else {
            ledStrip.setData(ledBuffer);
        }
    }
}
