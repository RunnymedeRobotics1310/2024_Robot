package frc.robot.subsystems.lighting;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.lighting.pattern.LightingPattern;
import frc.robot.telemetry.Telemetry1310;

public class LightingSubsystem extends SubsystemBase {

    private final AddressableLED       ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    private final LightstripRegion[]   regions;

    public LightingSubsystem(LightstripRegion... regions) {

        this.regions = regions;
        ledStrip     = new AddressableLED(Constants.LightingConstants.LIGHT_STRING_PWM_PORT);
        ledBuffer    = new AddressableLEDBuffer(Constants.LightingConstants.LIGHT_STRIP_LENGTH);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);

        ledStrip.start();
    }

    /**
     * Set the specified pattern as the only active pattern in its region
     */
    public void setPattern(LightingPattern pattern) {
        for (LightstripRegion region : regions) {
            if (pattern.getRegion() == region) {
                region.setPattern(pattern);
                break;
            }
        }
    }

    /**
     * Add a pattern to its region
     */
    public void addPattern(LightingPattern pattern) {
        for (LightstripRegion region : regions) {
            if (pattern.getRegion() == region) {
                region.addPattern(pattern);
                break;
            }
        }
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
            AddressableLEDBuffer buffer  = pattern.periodic();
            for (int i = 0; i < buffer.getLength(); i++) {
                ledBuffer.setLED(region.start + i, buffer.getLED(i));
            }

            Telemetry1310.light.regionStatus.put(region.name, pattern.getClass().getSimpleName());
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
