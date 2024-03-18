package frc.robot.subsystems.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

/**
 * Light signal to display for a few seconds when the robot is enabled. This pattern
 * will flash a fixed number of times once the RSL turns on for the first time.
 */
public class Enabled extends LightingPattern {

    private static final Color                RSL_COLOR = new Color(255, 20, 0);


    private static final AddressableLEDBuffer RSL_ON;
    private static final AddressableLEDBuffer RSL_OFF;
    private static final AddressableLEDBuffer RED_BUFFER;
    private static final AddressableLEDBuffer BLUE_BUFFER;
    static {

        // Static initializer for the RSL flash buffer
        RSL_ON      = new AddressableLEDBuffer(SIGNAL.length);
        RSL_OFF     = new AddressableLEDBuffer(SIGNAL.length);
        RED_BUFFER  = new AddressableLEDBuffer(SIGNAL.length);
        BLUE_BUFFER = new AddressableLEDBuffer(SIGNAL.length);

        for (int i = 0; i < SIGNAL.length; i++) {
            RSL_ON.setLED(i, RSL_COLOR);
            RSL_OFF.setLED(i, Color.kBlack);
            RED_BUFFER.setLED(i, Color.kRed);
            BLUE_BUFFER.setLED(i, Color.kFirstBlue);
        }
    }

    public static LightingPattern getInstance() {
        return new Enabled();
    }

    private int     rslFlashCount;
    private boolean prevRslState = false;

    private Enabled() {
        super(SIGNAL.length);
        rslFlashCount = 5;
    }

    @Override
    public AddressableLEDBuffer getBuffer() {

        if (rslFlashCount > 0) {
            return flashRSL();
        }
        else {
            if (getRunnymedeAlliance() == Red) {
                return RED_BUFFER;
            }
            else {
                return BLUE_BUFFER;
            }
        }
    }

    /**
     * Flash all LEDs in the region in time with the RSL light for a set number of flashes
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