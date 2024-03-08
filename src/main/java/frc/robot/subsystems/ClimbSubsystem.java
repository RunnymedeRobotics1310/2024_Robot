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

    private final CANSparkMax       leftClimbMotor        = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);

    private final CANSparkMax       rightClimbMotor       = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
//     private final DigitalInput linkLowerLimitSwitch = new
//     DigitalInput(Constants.ArmConstants.LINK_LOWER_LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput      rightClimbLimitSwitch = new DigitalInput(ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_2);
    private final DigitalInput      leftClimbLimitSwitch  = new DigitalInput(ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_3);

    private double                  rightClimbSpeed       = 0;
    private double                  leftClimbSpeed        = 0;
    private boolean                 safetyEnabled         = false;
    private long                    safetyStartTime       = 0;
    private boolean rightEncoderZeroed = false;
    private boolean leftEncoderZeroed = false;


    public ClimbSubsystem(LightingSubsystem lightingSubsystem) {
        this.lighting = lightingSubsystem;
    }

    public void setClimbSpeeds(double leftClimbSpeed, double rightClimbSpeed) {

        this.leftClimbSpeed  = leftClimbSpeed;
        this.rightClimbSpeed = rightClimbSpeed;

        checkClimbSafety();

        if (leftClimbSpeed > 0 || rightClimbSpeed > 0) {
            lighting.addPattern(Climbing.getInstance());
        } else {
            lighting.removePattern(Climbing.class);
        }

        leftClimbMotor.set(leftClimbSpeed);
        rightClimbMotor.set(rightClimbSpeed);
    }

    private boolean isRightClimbAtLimit() {
        return !rightClimbLimitSwitch.get();
    }

    private boolean isLeftClimbAtLimit() {
        return !leftClimbLimitSwitch.get();
    }

    public void stop() {
        setClimbSpeeds(0, 0);
    }

    @Override
    public void periodic() {

        if (rightClimbLimitSwitch.get() ) {
            rightEncoderZeroed = true;
            if (rightClimbMotor.getEncoder().getPosition() > 0) {
                rightClimbMotor.getEncoder().setPosition(0);
            }
        }

        if (leftClimbLimitSwitch.get()) {
            leftEncoderZeroed = true;
            if (leftClimbMotor.getEncoder().getPosition() > 0) {
                leftClimbMotor.getEncoder().setPosition(0);
            }
        }

        if (!rightEncoderZeroed) {
            rightClimbMotor.set(-ClimbConstants.SLOW_SPEED);
        }

        if (!leftEncoderZeroed) {
            leftClimbMotor.set(-ClimbConstants.SLOW_SPEED);
        }

        /*
         * Safety-check all of the motors speeds, and
         * set the motor outputs.
         *
         * This is required because a command may set the motor speed
         * at the beginning and may not ever set it again. The periodic
         * loop checks the limits every loop.
         */
        setClimbSpeeds(leftClimbSpeed, rightClimbSpeed);

        // Latch the climb safety for 2 seconds when a safety condition
        // is activated.
        if (safetyEnabled) {
            if ((System.currentTimeMillis() - safetyStartTime) > 2000) {
                safetyEnabled = false;
            }
        }

        /*
         * Update the SmartDashboard
         */

        Telemetry.climb.leftClimbSpeed    = leftClimbSpeed;
        Telemetry.climb.leftClimbEncoder  = leftClimbMotor.getEncoder().getPosition();
        Telemetry.climb.rightClimbSpeed   = rightClimbSpeed;
        Telemetry.climb.rightClimbEncoder = rightClimbMotor.getEncoder().getPosition();
        Telemetry.climb.safetyEnabled     = safetyEnabled;
        Telemetry.climb.rightLimit        = isRightClimbAtLimit();
        Telemetry.climb.leftLimit         = isLeftClimbAtLimit();
    }

    private void checkClimbSafety() {
        rightClimbSpeed = checkClimbSafety(rightClimbSpeed, rightClimbMotor.getEncoder().getPosition());
        leftClimbSpeed = checkClimbSafety(leftClimbSpeed, leftClimbMotor.getEncoder().getPosition());
    }

    private double checkClimbSafety(double climbSpeed, double climbEncoder) {

        // Do not allow the motor to go below the lower limit
        if (climbSpeed < 0 && climbEncoder <= ClimbConstants.CLIMB_MIN) {
            return 0;
        }

        // Do not extend further than the reach
        if (climbSpeed > 0 && climbEncoder > ClimbConstants.CLIMB_MAX) {
            return 0;
        }

        // Slow zones
        if (climbSpeed > ClimbConstants.SLOW_SPEED && (climbEncoder < ClimbConstants.BOTTOM_SLOW_ZONE || climbEncoder > ClimbConstants.TOP_SLOW_ZONE)) {
            return Math.signum(climbSpeed) * ClimbConstants.SLOW_SPEED;
        }

        return climbSpeed;
    }

}
