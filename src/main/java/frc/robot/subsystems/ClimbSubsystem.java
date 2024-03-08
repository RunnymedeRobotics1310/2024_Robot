package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.telemetry.Telemetry1310;

public class ClimbSubsystem extends SubsystemBase {

    // Lights Subsystem
    private final LightingSubsystem lighting;

    // private final CANSparkMax leftClimbMotor = new
    // CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_CAN_ADDRESS,
    // MotorType.kBrushless);

    // private final CANSparkMax rightClimbMotor = new
    // CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_CAN_ADDRESS,
    // MotorType.kBrushless);
    // private final DigitalInput linkLowerLimitSwitch = new
    // DigitalInput(ArmConstants.LINK_LOWER_LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput climbLimitSwitch2 = new DigitalInput(ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_2);
    private final DigitalInput climbLimitSwitch3 = new DigitalInput(ClimbConstants.CLIMB_LIMIT_SWITCH_DIO_PORT_3);

    private double             rightClimbSpeed   = 0;
    private double             leftClimbSpeed    = 0;

    private boolean            safetyEnabled     = false;
    private long               safetyStartTime   = 0;

    public ClimbSubsystem(LightingSubsystem lightingSubsystem) {
        this.lighting = lightingSubsystem;
    }

    public void setClimbSpeeds(double leftClimbSpeed, double rightClimbSpeed) {

        this.leftClimbSpeed  = leftClimbSpeed;
        this.rightClimbSpeed = rightClimbSpeed;

        checkClimbSafety();

        // leftClimbMotor.set(leftClimbSpeed);
        // rightClimbMotor.set(rightClimbSpeed);
    }

    public double getRightClimbEncoder() {
        // return rightClimbMotor.getEncoder().getPosition();
        return 0;
    }

    public double getLeftClimbEncoder() {
        // return leftClimbMotor.getEncoder().getPosition();
        return 0;
    }

    private boolean isClimbAtLimit2() {
        return !climbLimitSwitch2.get();
    }

    private boolean isClimbAtLimit3() {
        return !climbLimitSwitch3.get();
    }

    public void stop() {
        setClimbSpeeds(0, 0);
    }

    @Override
    public void periodic() {

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
        Telemetry1310.climb.leftClimbSpeed    = leftClimbSpeed;
        Telemetry1310.climb.leftClimbEncoder  = getLeftClimbEncoder();
        Telemetry1310.climb.rightClimbSpeed   = rightClimbSpeed;
        Telemetry1310.climb.rightClimbEncoder = getRightClimbEncoder();
        Telemetry1310.climb.safetyEnabled     = safetyEnabled;
        Telemetry1310.climb.isAtLimit2        = isClimbAtLimit2();
        Telemetry1310.climb.isAtLimit3        = isClimbAtLimit3();
    }

    private void checkClimbSafety() {

        // NOTE: Set safetyEnabled = true if a safety condition
        // is encountered

        leftClimbSpeed = checkClimbSafety(leftClimbSpeed, getLeftClimbEncoder());
        /*
         * RIGHT CLIMB RANGE
         *
         * if (rightClimbSpeed < 0 && rightClimbSpeedEncoder <= ClimbConstants.CLIMB_MIN) {
         * rightClimbSpeed = 0;
         * safetyEnabled = true;
         * safetyStartTime = System.currentTimeMillis();
         * System.out.println("right min");
         * }
         *
         * if (rightClimbSpeed > 0 && rightClimbSpeedEncoder <= ClimbConstants.CLIMB_MAX) {
         * rightClimbSpeed = 0;
         * safetyEnabled = true;
         * safetyStartTime = System.currentTimeMillis();
         * System.out.println("right max");
         * }
         *
         * /*
         * LEFT CLIMB RANGE
         *
         *
         * if (leftClimbSpeed < 0 && leftClimbSpeedEncoder <= ClimbConstants.CLIMB_MIN) {
         * leftClimbSpeed = 0;
         * safetyEnabled = true;
         * safetyStartTime = System.currentTimeMillis();
         * System.out.println("left min");
         * }
         *
         * if (leftClimbSpeed > 0 && leftClimbSpeedEncoder <= ClimbConstants.CLIMB_MAX) {
         * leftClimbSpeed = 0;
         * safetyEnabled = true;
         * safetyStartTime = System.currentTimeMillis();
         * System.out.println("left max");
         * }
         */

        return;
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
        return climbSpeed;
    }

}
