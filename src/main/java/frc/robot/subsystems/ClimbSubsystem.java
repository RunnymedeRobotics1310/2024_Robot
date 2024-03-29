package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.Constants.LightingConstants.SIGNAL;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Climbing;
import frc.robot.telemetry.Telemetry;

public class ClimbSubsystem extends RunnymedeSubsystemBase {

    // Lights Subsystem
    private final LightingSubsystem lighting;

    private final CANSparkMax       leftClimbMotor          = new CANSparkMax(
        ClimbConstants.LEFT_CLIMB_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);

    private final CANSparkMax       rightClimbMotor         = new CANSparkMax(
        ClimbConstants.RIGHT_CLIMB_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final DigitalInput      rightClimbLimitSwitch   = new DigitalInput(
        ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_2);
    private final DigitalInput      leftClimbLimitSwitch    = new DigitalInput(
        ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_3);

    private double                  rightClimbSpeed         = 0;
    private double                  leftClimbSpeed          = 0;

    private boolean                 rightEncoderInitialized = false;
    private boolean                 leftEncoderInitialized  = false;

    private boolean                 unsafeMode              = false;


    public ClimbSubsystem(LightingSubsystem lightingSubsystem) {

        this.lighting = lightingSubsystem;

    }

    public void setUnsafeMode(boolean unsafeMode) {
        this.unsafeMode = unsafeMode;
    }

    public boolean isRightClimbAtMax() {

        return rightClimbMotor.getEncoder().getPosition() >= ClimbConstants.CLIMB_MAX;
    }

    public boolean isLeftClimbAtMax() {
        return leftClimbMotor.getEncoder().getPosition() >= ClimbConstants.CLIMB_MAX;
    }

    @Override
    public void periodic() {

        if (unsafeMode || (leftEncoderInitialized && rightEncoderInitialized)) {

            // if everything is initialized, check safety then set motors to appropriate values.
            checkClimbSafety();
        }
        else {

            // init the encoders. Set motors speeds to appropriate values.
            initEncoders();
        }

        if (!DISABLED) {
            leftClimbMotor.set(leftClimbSpeed);
            rightClimbMotor.set(rightClimbSpeed);
        }

        setLightingPattern();

        Telemetry.climb.leftClimbSpeed    = leftClimbSpeed;
        Telemetry.climb.leftClimbEncoder  = leftClimbMotor.getEncoder().getPosition();
        Telemetry.climb.rightClimbSpeed   = rightClimbSpeed;
        Telemetry.climb.rightClimbEncoder = rightClimbMotor.getEncoder().getPosition();
        Telemetry.climb.rightLimit        = rightAllTheWayDown();
        Telemetry.climb.leftLimit         = leftAllTheWayDown();
        Telemetry.climb.leftInited        = leftEncoderInitialized;
        Telemetry.climb.rightInited       = rightEncoderInitialized;

    }

    public boolean leftAllTheWayDown() {
        return leftClimbLimitSwitch.get() == false;
    }

    public boolean climbersReadyForTrapShot() {
        return leftClimbMotor.getEncoder().getPosition() < CLIMB_READY_FOR_TRAP_SHOT && rightClimbMotor.getEncoder().getPosition() < CLIMB_READY_FOR_TRAP_SHOT;
    }

    public boolean rightAllTheWayDown() {
        return rightClimbLimitSwitch.get() == false;
    }

    public void setClimbSpeeds(double leftClimbSpeed, double rightClimbSpeed) {

        this.leftClimbSpeed  = leftClimbSpeed;
        this.rightClimbSpeed = rightClimbSpeed;

    }


    public void stop() {

        setClimbSpeeds(0, 0);

    }

    private void checkClimbSafety() {

        rightClimbSpeed = checkClimbSafety(rightClimbSpeed, rightClimbMotor.getEncoder().getPosition());
        leftClimbSpeed  = checkClimbSafety(leftClimbSpeed, leftClimbMotor.getEncoder().getPosition());

    }

    /**
     * Safety-check all of the motors speeds, and
     * set the motor outputs.
     *
     * This is required because a command may set the motor speed
     * at the beginning and may not ever set it again. The periodic
     * loop checks the limits every loop.
     */
    private double checkClimbSafety(double climbSpeed, double climbEncoder) {

        if (unsafeMode) {
            return climbSpeed;
        }

        // Do not allow the motor to go below the lower limit
        if (climbSpeed < 0 && climbEncoder <= ClimbConstants.CLIMB_MIN) {
            return 0;
        }

        // Do not extend further than the reach
        if (climbSpeed > 0 && climbEncoder > ClimbConstants.CLIMB_MAX) {
            return 0;
        }

        // Slow zones
        if (climbSpeed > 0) {
            // going up
            if (climbEncoder > TOP_SLOW_ZONE && climbSpeed > ClimbConstants.SLOW_SPEED) {
                return ClimbConstants.SLOW_SPEED;
            }
        }
        else {
            // going down
            if (climbEncoder < ClimbConstants.BOTTOM_SLOW_ZONE && climbSpeed < -ClimbConstants.SLOW_SPEED) {
                return -ClimbConstants.SLOW_SPEED;
            }
        }

        return climbSpeed;
    }

    public void initEncoders() {
        // log("Init encoders. Right:" + rightEncoderInitialized + " Left:" +
        // leftEncoderInitialized);
        if (!rightEncoderInitialized) {
            if (rightAllTheWayDown()) {
                rightClimbMotor.getEncoder().setPosition(0);
                rightClimbMotor.burnFlash();
                rightEncoderInitialized = true;
            }
            else {
                rightClimbSpeed = -ClimbConstants.INITIALIZE_CLIMBERS_SPEED;
            }
        }

        if (!leftEncoderInitialized) {
            if (leftAllTheWayDown()) {
                leftClimbMotor.getEncoder().setPosition(0);
                leftClimbMotor.burnFlash();
                leftEncoderInitialized = true;
            }
            else {
                leftClimbSpeed = -ClimbConstants.INITIALIZE_CLIMBERS_SPEED;
            }
        }
    }

    private void setLightingPattern() {
        if (rightEncoderInitialized && leftEncoderInitialized) {
            if (Math.abs(leftClimbSpeed) > 0 || Math.abs(rightClimbSpeed) > 0) {
                lighting.addPattern(SIGNAL, Climbing.getInstance());
            }
            else {
                lighting.removePattern(Climbing.class);
            }
        }
    }

}
