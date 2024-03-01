package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {

    // Note: on Shifty, the lights are Y-ed, both of the strips run the same pattern.

    private final AddressableLED              ledStrip;
    private final AddressableLEDBuffer        ledBuffer;

    private static final Color                NOTE_COLOR    = new Color(255, 20, 0);


    // RSL Flash
    private static final AddressableLEDBuffer RSL_ON;
    private static final AddressableLEDBuffer RSL_OFF;
    private static final Color                RSL_COLOR     = new Color(255, 20, 0);

    private int                               rslFlashCount = -1;
    private boolean                           prevRslState  = false;

    static {

        // Static initializer for the RSL flash buffer
        RSL_ON  = new AddressableLEDBuffer(LightsConstants.LIGHT_STRING_LENGTH);
        RSL_OFF = new AddressableLEDBuffer(LightsConstants.LIGHT_STRING_LENGTH);

        for (int i = 0; i < LightsConstants.LIGHT_STRING_LENGTH; i++) {
            RSL_ON.setLED(i, RSL_COLOR);
            RSL_OFF.setLED(i, Color.kBlack);
        }
    }

    public LightsSubsystem() {

        ledStrip  = new AddressableLED(LightsConstants.LIGHT_STRING_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LightsConstants.LIGHT_STRING_LENGTH);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);

        ledStrip.start();
    }

    @Override
    public void periodic() {

        if (rslFlashCount >= 0) {
            flashRSL();
        }
        else {
            ledStrip.setData(ledBuffer);
        }
    }

    public void setEnabled() {
        // Flash the RSL light when the robot is enabled
        rslFlashCount = 5;
    }

    /*
     * Arm LED routines
     */
    public void setLinkAngle(double linkAngle) {

        // TODO Use led 30 colour to indicate the position

        // Use the range 31-44 for the link angle.
        // Determine the LED to light
        // There are 14 leds spanning the range 0-130 deg ~ 9deg/led;
        //
        int led = (int) Math.round(linkAngle / 9);

        // Limit the LED to the range 0-13
        led = Math.min(13, led);
        led = Math.max(0, led);

        // Clear the display range
        clearLEDs(31, 44);

        // Set the LED indicating the arm angle
        ledBuffer.setLED(31 + led, Color.kGreen);
    }

    public void setAimAngle(double aimAngle) {

        // TODO Use led 29 colour to indicate the position

        // Use the range 15-28 for the aim angle.
        // Determine the LED to light
        // There are 14 leds spanning the range 60-190 deg ~ 9deg/led;
        //
        int led = (int) Math.round((aimAngle - 60) / 9);

        // Limit the LED to the range 0-13
        led = Math.min(13, led);
        led = Math.max(0, led);

        // Clear the display range
        clearLEDs(15, 28);

        // Set the LED indicating the aim angle
        // NOTE: LED 28 = 60, and LED 15 = 190, so the LEDs are inverted
        ledBuffer.setLED(28 - led, Color.kGreen);
    }


    public void setIntakeSpeed(double encoderIntakeSpeed, boolean intakeAtTargetSpeed) {

        Color color = new Color(0, 0, 0);

        if (encoderIntakeSpeed > 0) {

            color = new Color(0, 255, 0);

            if (!intakeAtTargetSpeed) {
                color = new Color(255, 0, 0);
            }
        }

        ledBuffer.setLED(1, color);
    }

    public void setShooterSpeed(double encoderShooterSpeed, boolean shooterAtTargetSpeed) {

        Color color = new Color(0, 0, 0);

        if (encoderShooterSpeed > 0) {

            color = new Color(0, 255, 0);

            if (!shooterAtTargetSpeed) {
                color = new Color(255, 0, 0);
            }
        }

        ledBuffer.setLED(0, color);
    }

    public void setNoteHeld(boolean noteHeld) {

        if (noteHeld) {
            ledBuffer.setLED(2, new Color(255, 20, 0));
        }
        else {
            ledBuffer.setLED(2, Color.kBlack);
        }
    }

    /**
     * Clear the range of LEDs
     *
     * @param start first LED to clear
     * @param end last LED to clear (note: this LED is included in the range)
     */
    private void clearLEDs(int start, int end) {

        for (int i = start; i <= end; i++) {
            ledBuffer.setLED(i, Color.kBlack);
        }
    }

    /**
     * Flash all LEDs in the buffer in time with the RSL light for a set number of flashes
     */
    private void flashRSL() {

        boolean rslState = RobotController.getRSLState();

        // when the RSL goes from on to off, decrement the flash count
        if (!rslState && prevRslState) {
            rslFlashCount--;
        }
        prevRslState = RobotController.getRSLState();

        if (rslState) {
            ledStrip.setData(RSL_ON);
        }
        else {
            ledStrip.setData(RSL_OFF);
        }
    }
}
