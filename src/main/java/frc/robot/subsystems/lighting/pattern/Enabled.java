package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

import static frc.robot.Constants.LightingConstants.SIGNAL;

public class Enabled extends LightingPattern {

    private static final Color                RSL_COLOR = new Color(255, 20, 0);
    private static final AddressableLEDBuffer RSL_ON;
    private static final AddressableLEDBuffer RSL_OFF;
    static {

        // Static initializer for the RSL flash buffer
        RSL_ON  = new AddressableLEDBuffer(Constants.LightingConstants.LIGHT_STRIP_LENGTH);
        RSL_OFF = new AddressableLEDBuffer(Constants.LightingConstants.LIGHT_STRIP_LENGTH);

        for (int i = 0; i < Constants.LightingConstants.LIGHT_STRIP_LENGTH; i++) {
            RSL_ON.setLED(i, RSL_COLOR);
            RSL_OFF.setLED(i, Color.kBlack);
        }
    }

    public static LightingPattern getInstance() {
        return new Enabled();
    }

    private int     rslFlashCount;
    private boolean prevRslState = false;

    private Enabled() {
        super(SIGNAL);
        rslFlashCount = 5;
    }

    @Override
    public AddressableLEDBuffer periodic() {

        if (rslFlashCount >= 0) {
            return flashRSL();
        }
        return RSL_OFF;

    }

    /**
     * Flash all LEDs in the buffer in time with the RSL light for a set number of flashes
     */
    private AddressableLEDBuffer flashRSL() {

        boolean rslState = RobotController.getRSLState();

        // when the RSL goes from on to off, decrement the flash count
        if (!rslState && prevRslState) {
            rslFlashCount--;
        }
        prevRslState = RobotController.getRSLState();

        if (rslState) {
            return RSL_ON;
        }
        else {
            return RSL_OFF;
        }
    }
}