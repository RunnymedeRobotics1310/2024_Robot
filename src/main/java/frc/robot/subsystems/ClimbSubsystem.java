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
    private final DigitalInput      rightClimbLimitSwitch = new DigitalInput(ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_2);
    private final DigitalInput      leftClimbLimitSwitch  = new DigitalInput(ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_3);

    private double                  rightClimbSpeed       = 0;
    private double                  leftClimbSpeed        = 0;
    private Double                  rightEncoderOffset    = null;
    private Double                  leftEncoderOffset     = null;


    public ClimbSubsystem(LightingSubsystem lightingSubsystem) {
        this.lighting = lightingSubsystem;
    }

    public void setClimbSpeeds(double leftClimbSpeed, double rightClimbSpeed) {

        this.leftClimbSpeed  = leftClimbSpeed;
        this.rightClimbSpeed = rightClimbSpeed;

        checkClimbSafety();

        if (rightEncoderOffset != null && leftEncoderOffset != 0) {
            if (Math.abs(leftClimbSpeed) > 0 || Math.abs(rightClimbSpeed) > 0) {
                lighting.addPattern(Climbing.getInstance());
            }
            else {
                lighting.removePattern(Climbing.class);
            }
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

    private double getLeftEncoderPos() {
        if (leftEncoderOffset == null)
            return 0;

        return leftClimbMotor.getEncoder().getPosition() - leftEncoderOffset.doubleValue();
    }

    private double getRightEncoderPos() {
        if (rightEncoderOffset == null)
            return 0;

        return rightClimbMotor.getEncoder().getPosition() - rightEncoderOffset.doubleValue();
    }

    @Override
    public void periodic() {

        if (rightEncoderOffset == null) {
            if (isRightClimbAtLimit()) {
                rightEncoderOffset = rightClimbMotor.getEncoder().getPosition();
            }
            else {
                rightClimbMotor.set(-ClimbConstants.SLOW_SPEED);
            }
        }

        if (leftEncoderOffset == null) {
            if (isLeftClimbAtLimit()) {
                leftEncoderOffset = leftClimbMotor.getEncoder().getPosition();
            }
            else {
                leftClimbMotor.set(-ClimbConstants.SLOW_SPEED);
            }
        }

        if (leftEncoderOffset == null || rightEncoderOffset == null) {
            // nothing else happens till we reset encoders.
            return;
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

        /*
         * Update the SmartDashboard
         */

        Telemetry.climb.leftClimbSpeed    = leftClimbSpeed;
        Telemetry.climb.leftClimbEncoder  = getLeftEncoderPos();
        Telemetry.climb.rightClimbSpeed   = rightClimbSpeed;
        Telemetry.climb.rightClimbEncoder = getRightEncoderPos();
        Telemetry.climb.rightLimit        = isRightClimbAtLimit();
        Telemetry.climb.leftLimit         = isLeftClimbAtLimit();

    }

    private void checkClimbSafety() {
        rightClimbSpeed = checkClimbSafety(rightClimbSpeed, getRightEncoderPos());
        leftClimbSpeed  = checkClimbSafety(leftClimbSpeed, getLeftEncoderPos());
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
        if (climbSpeed > ClimbConstants.SLOW_SPEED
            && (climbEncoder < ClimbConstants.BOTTOM_SLOW_ZONE || climbEncoder > ClimbConstants.TOP_SLOW_ZONE)) {
            return Math.signum(climbSpeed) * ClimbConstants.SLOW_SPEED;
        }

        return climbSpeed;
    }

}
