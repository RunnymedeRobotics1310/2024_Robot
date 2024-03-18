package frc.robot.subsystems.lighting;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.RunnymedeSubsystemBase;
import frc.robot.subsystems.lighting.pattern.LightingPattern;
import frc.robot.telemetry.Telemetry;

import java.util.ArrayList;

public class LightingSubsystem extends RunnymedeSubsystemBase {

    private final AddressableLED       ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    private final LightstripRegion[]   regions;

    public LightingSubsystem(LightstripRegion... regions) {

        this.regions = regions;
        ledStrip     = new AddressableLED(Constants.LightingConstants.PWM_PORT);
        ledBuffer    = new AddressableLEDBuffer(Constants.LightingConstants.STRIP_LENGTH);

        validateConfiguration(regions);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);

        ledStrip.start();
    }

    /**
     * Set the specified pattern as the only active pattern in its region
     */
    public void setPattern(LightstripRegion region, LightingPattern pattern) {
        region.setPattern(pattern);
    }

    /**
     * Add a pattern to its region
     */
    public void addPattern(LightstripRegion region, LightingPattern pattern) {
        region.addPattern(pattern);
    }

    /**
     * Remove the specified pattern from its region.
     */
    public void removePattern(Class<? extends LightingPattern> patternClass) {
        for (LightstripRegion region : regions) {
            region.removePattern(patternClass);
        }
    }


    @Override
    public void periodic() {
        // set the lights in each region
        for (LightstripRegion region : regions) {

            LightingPattern      pattern = region.getPattern();
            AddressableLEDBuffer buffer  = pattern.getBuffer();
            for (int i = 0; i < buffer.getLength(); i++) {
                ledBuffer.setLED(region.start + i, buffer.getLED(i));
            }

            Telemetry.light.regionStatus.put(region.name, pattern.getClass().getSimpleName());
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
            log("Dimming lights to reduce current draw");
            ledStrip.setData(dimmedBuffer);
        }
        else {
            ledStrip.setData(ledBuffer);
        }
    }

    private static void validateConfiguration(LightstripRegion... regions) {
        ArrayList<Boolean> check = new ArrayList<>(Constants.LightingConstants.STRIP_LENGTH);
        for (int i = 0; i < Constants.LightingConstants.STRIP_LENGTH; i++) {
            check.add(false);
        }

        for (LightstripRegion region : regions) {
            for (int i = region.start; i < region.start + region.length; i++) {
                if (check.get(i)) {
                    throw new IllegalArgumentException("Lightstrip regions overlap at index " + i);
                }
                check.set(i, true);
            }
        }
        for (int i = 0; i < check.size(); i++) {
            if (!check.get(i)) {
                throw new IllegalArgumentException(
                    "Lightstrip regions do not cover the entire lightstrip. No value found for LED " + i);
            }
        }
    }
}
