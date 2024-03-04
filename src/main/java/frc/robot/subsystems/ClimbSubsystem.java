package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    // Lights Subsystem
    private final LightsSubsystem lightsSubsystem;

    private final CANSparkMax     leftClimbMotor  = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);

    private final CANSparkMax     rightClimbMotor = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);

    private double                rightClimbSpeed = 0;
    private double                leftClimbSpeed  = 0;

    private boolean               safetyEnabled   = false;
    private long                  safetyStartTime = 0;

    public ClimbSubsystem(LightsSubsystem lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
    }

    public void setClimbSpeeds(double leftClimbSpeed, double rightClimbSpeed) {

        this.leftClimbSpeed  = leftClimbSpeed;
        this.rightClimbSpeed = rightClimbSpeed;

        checkClimbSafety();

        leftClimbMotor.set(leftClimbSpeed);
        rightClimbMotor.set(rightClimbSpeed);
    }

    public double getRightClimbEncoder() {
        return rightClimbMotor.getEncoder().getPosition();
    }

    public double getLeftClimbEncoder() {
        return leftClimbMotor.getEncoder().getPosition();
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

        SmartDashboard.putNumber("Left Climb Speed", leftClimbSpeed);
        SmartDashboard.putNumber("Left Climb Encoder", getLeftClimbEncoder());

        SmartDashboard.putNumber("Right Climb Speed", rightClimbSpeed);
        SmartDashboard.putNumber("Right Climb Encoder", getRightClimbEncoder());

        SmartDashboard.putBoolean("Climb Safety", safetyEnabled);

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
