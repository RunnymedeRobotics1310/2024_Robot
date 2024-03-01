package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private final LightsSubsystem lightsSubsystem;

    private final CANSparkMax     linkMotor           = new CANSparkMax(ArmConstants.LINK_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax     aimMotor            = new CANSparkMax(ArmConstants.AIM_MOTOR_CAN_ADDRESS, MotorType.kBrushless);

    private final CANSparkMax     intakeMotor         = new CANSparkMax(ArmConstants.AIM_MOTOR_CAN_ADDRESS, MotorType.kBrushless);
    private final CANSparkMax     shooterBottomMotor  = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax     shooterTopMotor     = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS + 1,
        MotorType.kBrushless);

    private AnalogInput           linkAbsoluteEncoder = new AnalogInput(ArmConstants.LINK_ENCODER_ANALOG_PORT);

    private double                linkPivotSpeed      = 0;
    private double                aimPivotSpeed       = 0;
    private double                intakeSpeed         = 0;
    private double                shooterSpeed        = 0;

    private double                aimAngleEncoder     = 0;

    private DigitalInput          noteDetector        = new DigitalInput(1);

    private boolean               safetyEnabled       = false;
    private long                  safetyStartTime     = 0;

    private double                linkEncoderOffset   = 0;

    public ArmSubsystem(LightsSubsystem lightsSubsystem) {

        this.lightsSubsystem = lightsSubsystem;

        // This is faking an angle encoder
        this.aimAngleEncoder = ArmConstants.COMPACT_ARM_POSITION.aimAngle;
    }

    public double getAimAngle() {
        return aimAngleEncoder;
    }

    public double getLinkEncoderVoltage() {
        // 0-5V range = 0-360 deg
        return linkAbsoluteEncoder.getVoltage();
    }

    public double getLinkAngle() {
        // 0-5V range = 0-360 deg
        return (getLinkEncoderVoltage() * 360.0 / 5.0 + linkEncoderOffset) % 360;
    }

    public double getShooterEncoderSpeed() {
        return shooterBottomMotor.getEncoder().getVelocity();
    }

    public double getIntakeEncoderSpeed() {
        return intakeMotor.getEncoder().getVelocity();
    }

    public boolean isNoteDetected() {
        return !noteDetector.get();
    }

    public void setLinkPivotSpeed(double speed) {
        this.linkPivotSpeed = speed;

        checkArmSafety();

        // linkPivotMotor.setSpeed(linkPivotSpeed);
    }

    public void setAimPivotSpeed(double speed) {

        this.aimPivotSpeed = speed;

        checkArmSafety();
        // linkAimMotor.setSpeed(linkAimSpeed);

    }

    public void setIntakeSpeed(double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        intakeMotor.set(intakeSpeed);
    }

    public void setShooterSpeed(double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
        shooterTopMotor.set(shooterSpeed);
        shooterBottomMotor.set(shooterSpeed);
    }

    public void stop() {
        setLinkPivotSpeed(0);
        setAimPivotSpeed(0);
        setIntakeSpeed(0);
        setShooterSpeed(0);
    }

    @Override
    public void periodic() {

        // TODO REMOVE this code when the real robot is available

        // FAKE THE MOTOR ACTION

        // Move the aim and link angles based on the speed.
        aimAngleEncoder = Math.min(200, Math.max(0, aimAngleEncoder + aimPivotSpeed));

        // END TODO Code removal

        /*
         * Safety-check all of the motors speeds, and
         * set the motor outputs.
         *
         * This is required because a command may set the motor speed
         * at the beginning and may not ever set it again. The periodic
         * loop checks the limits every loop.
         */
        setLinkPivotSpeed(linkPivotSpeed);
        setAimPivotSpeed(aimPivotSpeed);

        // Latch the arm safety for 2 seconds when a safety condition
        // is activated.
        if (safetyEnabled) {
            if ((System.currentTimeMillis() - safetyStartTime) > 2000) {
                safetyEnabled = false;
            }
        }

        /*
         * Update the SmartDashboard
         */

        SmartDashboard.putNumber("Intake Motor", intakeSpeed);
        SmartDashboard.putNumber("Intake Encoder Speed", getIntakeEncoderSpeed());

        SmartDashboard.putNumber("Shooter Motor", shooterSpeed);
        SmartDashboard.putNumber("Shooter Encoder Speed", getShooterEncoderSpeed());

        SmartDashboard.putNumber("Link Speed", linkPivotSpeed);
        SmartDashboard.putNumber("Link Angle", getLinkAngle());
        SmartDashboard.putNumber("Link Absolute Encoder Voltage", getLinkEncoderVoltage());

        SmartDashboard.putNumber("Aim Speed", aimPivotSpeed);
        SmartDashboard.putNumber("Aim Angle", getAimAngle());

        SmartDashboard.putBoolean("Note Detected", isNoteDetected());

        SmartDashboard.putBoolean("Arm Safety", safetyEnabled);

        // Update the lights
        updateLights();
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append("Link ").append(getLinkAngle()).append("deg (").append(linkPivotSpeed).append(") ")
            .append("Aim ").append(getAimAngle()).append("deg (").append(aimPivotSpeed).append(") ")
            .append("Intake ").append(intakeSpeed).append(", ").append(getIntakeEncoderSpeed()).append(' ')
            .append("Shooter ").append(shooterSpeed).append(", ").append(getShooterEncoderSpeed()).append(' ')
            .append("Game Piece ").append(isNoteDetected());

        return sb.toString();
    }

    /*
     * Update the lights based on the current state of the arm
     * FIXME: the lighting pattern is specific to Shifty.
     */
    private void updateLights() {

        /*
         * lightsSubsystem.setIntakeSpeed(intakeSpeedEncoder, intakeAtTargetSpeed);
         * 
         * boolean shooterAtTargetSpeed = Math.abs(shooterSpeed - shooterSpeedEncoder) < .05;
         * 
         * lightsSubsystem.setShooterSpeed(shooterSpeedEncoder, shooterAtTargetSpeed);
         * 
         * lightsSubsystem.setLinkAngle(getLinkAngle());
         * 
         * lightsSubsystem.setAimAngle(getAimAngle());
         * 
         * lightsSubsystem.setNoteHeld(isNoteDetected());
         * 
         */
    }

    private void checkArmSafety() {

        // NOTE: Set safetyEnabled = true if a safety condition
        // is encountered

        // TODO:

        /*
         * LINK RANGE
         */
        /*
         * Never drive the link lower than the hard stop.
         *
         * If the link lower limit switch is active, then stop lowering
         * the link.
         */
        if (linkPivotSpeed < 0 && getLinkAngle() <= ArmConstants.LINK_MIN_DEGREES) {
            linkPivotSpeed  = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * Never drive the link past 125 deg.
         *
         * The arm never needs to be that high.
         */
        if (linkPivotSpeed > 0 && getLinkAngle() >= ArmConstants.LINK_MAX_DEGREES) {
            linkPivotSpeed  = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * AIM RANGE
         */
        /*
         * Never drive the aim angle > 200 deg.
         *
         * The aim never needs to be that high.
         */
        if (aimPivotSpeed > 0 && aimAngleEncoder >= ArmConstants.AIM_MAX_DEGREES) {
            aimPivotSpeed   = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * Never drive the aim angle < 60 deg.
         *
         * The aim never needs to be that low.
         */
        if (aimPivotSpeed < 0 && aimAngleEncoder <= ArmConstants.AIM_MIN_DEGREES) {
            aimPivotSpeed   = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * TOTAL ARM ANGLES
         */
        /*
         * Never drive the motors to a total of less than 180 degrees when
         * the arm is inside the frame Link angle < over bumper position
         *
         * Strategy:
         * Turn off the motor that is lowering the total arm angle.
         * Allow any positive movements to continue.
         */
        if (aimAngleEncoder + getLinkAngle() <= ArmConstants.ARM_MIN_ANGLE_SUM) {
            if (aimPivotSpeed < 0) {
                aimPivotSpeed   = 0;
                safetyEnabled   = true;
                safetyStartTime = System.currentTimeMillis();
            }
            if (linkPivotSpeed < 0) {
                linkPivotSpeed  = 0;
                safetyEnabled   = true;
                safetyStartTime = System.currentTimeMillis();
            }
        }


        /*
         * Never drive the motors to a total of more xxx degrees when
         * the arm is near the 4ft limit (arm angle > xxx).
         * Link angle > xxx position
         *
         * Strategy:
         * Turn off the motor that is raising the total arm angle.
         * Allow any negative movements to continue.
         */

        if (aimAngleEncoder + getLinkAngle() >= ArmConstants.ARM_MAX_ANGLE_SUM) {
            if (aimPivotSpeed > 0) {
                aimPivotSpeed   = 0;
                safetyEnabled   = true;
                safetyStartTime = System.currentTimeMillis();
            }
            if (linkPivotSpeed > 0) {
                linkPivotSpeed  = 0;
                safetyEnabled   = true;
                safetyStartTime = System.currentTimeMillis();
            }
        }
        return;
    }

    public void setArmSpeeds(double linkSpeed, double aimSpeed) {

        setLinkPivotSpeed(linkSpeed);
        setAimPivotSpeed(aimSpeed);

    }
}
