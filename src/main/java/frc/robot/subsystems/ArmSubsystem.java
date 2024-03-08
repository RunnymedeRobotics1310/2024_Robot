package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.lighting.LightingSubsystem;


public class ArmSubsystem extends SubsystemBase {

    private final LightingSubsystem lighting;

    private final CANSparkMax       linkMotor            = new CANSparkMax(ArmConstants.LINK_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax       aimMotor             = new CANSparkMax(ArmConstants.AIM_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);

    private final CANSparkMax       intakeMotor          = new CANSparkMax(ArmConstants.INTAKE_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax       shooterBottomMotor   = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax       shooterTopMotor      = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS + 1,
        MotorType.kBrushless);

    private final DigitalInput      linkLowerLimitSwitch = new DigitalInput(ArmConstants.LINK_LOWER_LIMIT_SWITCH_DIO_PORT);

    private final DigitalInput      noteDetector         = new DigitalInput(ArmConstants.INTAKE_NOTE_DETECTOR_DIO_PORT);

    private final AnalogInput       linkAbsoluteEncoder  = new AnalogInput(ArmConstants.LINK_ABSOLUTE_ENCODER_ANALOG_PORT);
    private final AnalogInput       aimAbsoluteEncoder   = new AnalogInput(ArmConstants.AIM_ABSOLUTE_ENCODER_ANALOG_PORT);

    private double                  linkPivotSpeed       = 0;
    private double                  aimPivotSpeed        = 0;
    private double                  intakeSpeed          = 0;
    private double                  shooterSpeed         = 0;

    private boolean                 safetyEnabled        = false;
    private long                    safetyStartTime      = 0;

    private boolean                 armSafetyMode        = true;

    public ArmSubsystem(LightingSubsystem lightingSubsystem) {

        this.lighting = lightingSubsystem;
    }


    private double getAimAbsoluteEncoderVoltage() {
        // 0-5V range = 0-360 deg
        return aimAbsoluteEncoder.getVoltage();
    }

    // todo: fixme: return rotation2d or specify unit in either javadoc or method name
    public double getAimAngle() {

        // The aim encoder should wrap at 4V.
        double aimEncoderVoltage = getAimAbsoluteEncoderVoltage();

        if (aimEncoderVoltage > 4) {
            aimEncoderVoltage -= 5.0;
        }

        // The conversion from volts to degrees
        double angle = aimEncoderVoltage
            * ArmConstants.AIM_ABSOLUTE_ENCODER_DEG_PER_VOLT
            + ArmConstants.AIM_ABSOLUTE_ENCODER_OFFSET_DEG;

        // round to 2 decimal places
        return Math.round(angle * 100) / 100.0d;
    }

    private double getLinkAbsoluteEncoderVoltage() {
        // 0-5V range = 0-360 deg
        return linkAbsoluteEncoder.getVoltage();
    }

    // todo: fixme: return rotation2d or specify unit in either javadoc or method name
    public double getLinkAngle() {

        // The conversion from volts to degrees
        double angle = getLinkAbsoluteEncoderVoltage()
            * ArmConstants.LINK_ABSOLUTE_ENCODER_DEG_PER_VOLT
            + ArmConstants.LINK_ABSOLUTE_ENCODER_OFFSET_DEG;

        // round to 2 decimal places
        return Math.round(angle * 100) / 100.0d;
    }

    // todo: fixme: specify unit in either javadoc or method name
    public double getShooterEncoderSpeed() {
        return shooterBottomMotor.getEncoder().getVelocity();
    }

    // todo: fixme: specify unit in either javadoc or method name
    public double getIntakeEncoderSpeed() {
        return Math.round(intakeMotor.getEncoder().getVelocity() * 100) / 100.0;
    }

    // This relies on 206.3 being the link angle. This should probably be changed to use different
    // link angles as well.
    // todo: fixme: specify unit in either javadoc or method name
    public Translation2d getShooterXY() {
        double shooterAngle = getAimAngle() - 90;



        double yDifference  = 0.20955 * (Math.sin(shooterAngle));
        double xDifference  = 0.20955 * (Math.sin(90 - shooterAngle));



        double shooterX     = ArmConstants.AIM_X_SHOOTING - xDifference;
        double shooterY     = ArmConstants.AIM_Y_SHOOTING + yDifference;



        return new Translation2d(shooterX, shooterY);

    }

    private boolean isLinkAtLowerLimit() {
        return !linkLowerLimitSwitch.get();
    }



    public boolean isNoteDetected() {
        return noteDetector.get();
    }

    // todo: fixme: specify units in either javadoc or param names (e.g. linkSpeedPercent or
    // linkSpeedRPM, etc)
    public void setArmPivotTestSpeeds(double linkSpeed, double aimSpeed) {

        armSafetyMode       = false;

        this.linkPivotSpeed = linkSpeed;
        this.aimPivotSpeed  = aimSpeed;

        linkMotor.set(linkSpeed);
        aimMotor.set(aimSpeed);
    }

    // todo: fixme: specify units in either javadoc or param names (e.g. linkSpeedPercent or
    // linkSpeedRPM, etc)
    public void setLinkPivotSpeed(double speed) {

        this.linkPivotSpeed = -speed;

        checkArmSafety();

        if (!ArmConstants.DISABLE_LINK) {
            linkMotor.set(linkPivotSpeed);
        }
    }

    // todo: fixme: specify units in either javadoc or param names (e.g. linkSpeedPercent or
    // linkSpeedRPM, etc)
    public void setAimPivotSpeed(double speed) {

        this.aimPivotSpeed = speed;

        checkArmSafety();

        if (!ArmConstants.DISABLE_AIM) {
            aimMotor.set(aimPivotSpeed);
        }
    }

    // todo: fixme: specify units in either javadoc or param names (e.g. linkSpeedPercent or
    // linkSpeedRPM, etc)
    public void setIntakeSpeed(double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        intakeMotor.set(intakeSpeed);
    }

    // todo: fixme: specify units in either javadoc or param names (e.g. linkSpeedPercent or
    // linkSpeedRPM, etc)
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
        SmartDashboard.putNumber("Aim Absolute Encoder Voltage", getAimAbsoluteEncoderVoltage());

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
