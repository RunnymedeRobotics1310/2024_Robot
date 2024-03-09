package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Climbing;
import frc.robot.telemetry.Telemetry;

public class ClimbSubsystem extends SubsystemBase {

    // Lights Subsystem
    private final LightingSubsystem lighting;

    private final CANSparkMax       leftClimbMotor          = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);

    private final CANSparkMax       rightClimbMotor         = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final DigitalInput      rightClimbLimitSwitch   = new DigitalInput(ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_2);
    private final DigitalInput      leftClimbLimitSwitch    = new DigitalInput(ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_3);

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

    public void setClimbSpeeds(double leftClimbSpeed, double rightClimbSpeed) {

        this.leftClimbSpeed  = leftClimbSpeed;
        this.rightClimbSpeed = rightClimbSpeed;

        if (!unsafeMode && leftEncoderInitialized && rightEncoderInitialized) {

            // if everything is initialized, check safety then turn the motors
            checkClimbSafety();

            leftClimbMotor.set(leftClimbSpeed);
            rightClimbMotor.set(rightClimbSpeed);
        }
        else {

            // init the encoders.
            initEncoders();
        }

    }


    public void stop() {

        setClimbSpeeds(0, 0);

    }

    @Override
    public void periodic() {

        setClimbSpeeds(leftClimbSpeed, rightClimbSpeed);

        setLightingPattern();

        Telemetry.climb.leftClimbSpeed    = leftClimbSpeed;
        Telemetry.climb.leftClimbEncoder  = leftClimbMotor.getEncoder().getPosition();
        Telemetry.climb.rightClimbSpeed   = rightClimbSpeed;
        Telemetry.climb.rightClimbEncoder = rightClimbMotor.getEncoder().getPosition();
        Telemetry.climb.rightLimit        = rightClimbLimitSwitch.get();
        Telemetry.climb.leftLimit         = leftClimbLimitSwitch.get();

    }

    private void checkClimbSafety() {

        rightClimbSpeed = checkClimbSafety(rightClimbSpeed, leftClimbMotor.getEncoder().getPosition());
        leftClimbSpeed  = checkClimbSafety(leftClimbSpeed, rightClimbMotor.getEncoder().getPosition());

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
        if (climbSpeed > ClimbConstants.SLOW_SPEED
            && (climbEncoder < ClimbConstants.BOTTOM_SLOW_ZONE || climbEncoder > ClimbConstants.TOP_SLOW_ZONE)) {
            return Math.signum(climbSpeed) * ClimbConstants.SLOW_SPEED;
        }

        return climbSpeed;
    }

    public void initEncoders() {
        if (!rightEncoderInitialized) {
            System.out.println("Zeroing right climb encoder. Limit: " + rightClimbLimitSwitch.get());
            if (!rightClimbLimitSwitch.get()) {
                rightClimbMotor.getEncoder().setPosition(0);
                rightClimbMotor.burnFlash();
                rightEncoderInitialized = true;
            }
            else {
                rightClimbMotor.set(-ClimbConstants.SLOW_SPEED);
            }
        }

        if (!leftEncoderInitialized) {
            System.out.println("Zeroing left climb encoder. Limit:" + leftClimbLimitSwitch.get());
            if (!leftClimbLimitSwitch.get()) {
                leftClimbMotor.getEncoder().setPosition(0);
                leftClimbMotor.burnFlash();
                leftEncoderInitialized = true;
            }
            else {
                leftClimbMotor.set(-ClimbConstants.SLOW_SPEED);
            }
        }
    }

    private void setLightingPattern() {
        if (rightEncoderInitialized && leftEncoderInitialized) {
            if (Math.abs(leftClimbSpeed) > 0 || Math.abs(rightClimbSpeed) > 0) {
                lighting.addPattern(Climbing.getInstance());
            }
            else {
                lighting.removePattern(Climbing.class);
            }
        }
    }

}
