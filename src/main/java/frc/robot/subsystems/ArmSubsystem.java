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

    private final CANSparkMax     linkMotor            = new CANSparkMax(ArmConstants.LINK_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax     aimMotor             = new CANSparkMax(ArmConstants.AIM_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);

    private final CANSparkMax     intakeMotor          = new CANSparkMax(ArmConstants.INTAKE_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax     shooterBottomMotor   = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax     shooterTopMotor      = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS + 1,
        MotorType.kBrushless);

    private DigitalInput          linkLowerLimitSwitch = new DigitalInput(ArmConstants.LINK_LOWER_LIMIT_SWITCH_DIO_PORT);

    private DigitalInput          noteDetector         = new DigitalInput(ArmConstants.INTAKE_NOTE_DETECTOR_DIO_PORT);

    private AnalogInput           linkAbsoluteEncoder  = new AnalogInput(ArmConstants.LINK_ABSOLUTE_ENCODER_ANALOG_PORT);
    private AnalogInput           aimAbsoluteEncoder   = new AnalogInput(ArmConstants.AIM_ABSOLUTE_ENCODER_ANALOG_PORT);

    private double                linkPivotSpeed       = 0;
    private double                aimPivotSpeed        = 0;
    private double                intakeSpeed          = 0;
    private double                shooterSpeed         = 0;

    private boolean               safetyEnabled        = false;
    private long                  safetyStartTime      = 0;

    private boolean               armSafetyMode        = true;

    public ArmSubsystem(LightsSubsystem lightsSubsystem) {

        this.lightsSubsystem = lightsSubsystem;
    }


    public double getAimAbsoluteEncoderVoltage() {
        // 0-5V range = 0-360 deg
        return aimAbsoluteEncoder.getVoltage();
    }

    public double getAimAngle() {

        // Get the sensor angle
        double rawAngle = getAimAbsoluteEncoderVoltage() * 360.0 / 5.0;

        // The aim angle is scaled by the gear ratio between the sensor and the shaft
        // and the offset to measure the aim angle relative to the link
        double hexShaftAngle = rawAngle * ArmConstants.AIM_ABSOLUTE_ENCODER_SCALING_FACTOR
            + ArmConstants.AIM_ABSOLUTE_ENCODER_OFFSET;

        // Adjust to 0-360 range
        hexShaftAngle %= 360;

        // round to 2 decimal places
        return Math.round(hexShaftAngle * 100) / 100.0d;
    }

    public double getLinkAbsoluteEncoderVoltage() {
        // 0-5V range = 0-360 deg
        return linkAbsoluteEncoder.getVoltage();
    }

    public double getLinkAngle() {

        double rawAngle      = getLinkAbsoluteEncoderVoltage() * 360.0 / 5.0;

        // The link angle is scaled by the gear ratio between the sensor and the shaft
        // and the offset to measure the link angle relative to the vertical from ground
        // (90deg = parallel to ground)
        double hexShaftAngle = rawAngle * ArmConstants.LINK_ABSOLUTE_ENCODER_SCALING_FACTOR
            + ArmConstants.LINK_ABSOLUTE_ENCODER_OFFSET;

        // Adjust to 0-360 range
        hexShaftAngle %= 360;

        // round to 2 decimal places
        return Math.round(hexShaftAngle * 100) / 100.0d;
    }

    public double getShooterEncoderSpeed() {
        return shooterBottomMotor.getEncoder().getVelocity();
    }

    public double getIntakeEncoderSpeed() {
        return Math.round(intakeMotor.getEncoder().getVelocity() * 100) / 100.0;
    }

    public boolean isLinkAtLowerLimit() {
        return !linkLowerLimitSwitch.get();
    }

    public boolean isNoteDetected() {
        return noteDetector.get();
    }

    public void setArmPivotTestSpeeds(double linkSpeed, double aimSpeed) {

        armSafetyMode       = false;

        this.linkPivotSpeed = linkSpeed;
        this.aimPivotSpeed  = aimSpeed;

        linkMotor.set(linkSpeed);
        aimMotor.set(aimSpeed);
    }

    public void setLinkPivotSpeed(double speed) {

        this.linkPivotSpeed = speed;

        checkArmSafety();

        linkMotor.set(linkPivotSpeed);
    }

    public void setAimPivotSpeed(double speed) {

        this.aimPivotSpeed = speed;

        checkArmSafety();

        aimMotor.set(aimPivotSpeed);
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

        /*
         * Safety-check all of the motors speeds, and
         * set the motor outputs.
         *
         * This is required because a command may set the motor speed
         * at the beginning and may not ever set it again. The periodic
         * loop checks the limits every loop.
         *
         * Safety can be bypassed by the test mode commands
         */
        if (armSafetyMode) {

            setLinkPivotSpeed(linkPivotSpeed);
            setAimPivotSpeed(aimPivotSpeed);

            // Latch the arm safety for 2 seconds when a safety condition
            // is activated.
            if (safetyEnabled) {
                if ((System.currentTimeMillis() - safetyStartTime) > 2000) {
                    safetyEnabled = false;
                }
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
        SmartDashboard.putNumber("Link Absolute Encoder Voltage", getLinkAbsoluteEncoderVoltage());
        SmartDashboard.putBoolean("Link Lower Limit", isLinkAtLowerLimit());

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
            .append(isLinkAtLowerLimit() ? "LINK LOWER LIMIT" : "")
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

        armSafetyMode = true;

        // NOTE: Set safetyEnabled = true if a safety condition
        // is encountered

        /*
         * LINK RANGE
         */
        /*
         * Never drive the link lower than the hard stop.
         *
         * If the link lower limit switch is active, then stop lowering
         * the link.
         */
        if (linkPivotSpeed < 0 && getLinkAngle() <= ArmConstants.LINK_MIN_DEGREES
            || isLinkAtLowerLimit()) {
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
        if (aimPivotSpeed > 0 && getAimAngle() >= ArmConstants.AIM_MAX_DEGREES) {
            aimPivotSpeed   = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * Never drive the aim angle < 60 deg.
         *
         * The aim never needs to be that low.
         */
        if (aimPivotSpeed < 0 && getAimAngle() <= ArmConstants.AIM_MIN_DEGREES) {
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
        if (getAimAngle() + getLinkAngle() <= ArmConstants.ARM_MIN_ANGLE_SUM) {
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

        if (getAimAngle() + getLinkAngle() >= ArmConstants.ARM_MAX_ANGLE_SUM) {
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

}
