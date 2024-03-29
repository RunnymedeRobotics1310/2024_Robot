package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ArmConstants;
import frc.robot.telemetry.Telemetry;

import java.util.function.Supplier;

import static frc.robot.Constants.ArmConstants.*;


public class ArmSubsystem extends RunnymedeSubsystemBase {
    private final CANSparkMax        linkMotor                = new CANSparkMax(ArmConstants.LINK_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax        aimMotor                 = new CANSparkMax(ArmConstants.AIM_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax        intakeMotor              = new CANSparkMax(ArmConstants.INTAKE_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax        shooterBottomMotor       = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax        shooterTopMotor          = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS + 1,
        MotorType.kBrushless);
    private final int                maxSparkMaxConfigRetries = 5;
    private final RelativeEncoder    intakeEncoder;
    private final RelativeEncoder    shooterTopEncoder;
    private final RelativeEncoder    shooterBottomEncoder;
    private final SparkPIDController intakePid;
    private final SparkPIDController shooterTopPid;
    private final SparkPIDController shooterBottomPid;
    private final DigitalInput       linkLowerLimitSwitch     = new DigitalInput(ArmConstants.LINK_LOWER_LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput       noteDetector             = new DigitalInput(ArmConstants.INTAKE_NOTE_DETECTOR_DIO_PORT);
    private final AnalogInput        linkAbsoluteEncoder      = new AnalogInput(ArmConstants.LINK_ABSOLUTE_ENCODER_ANALOG_PORT);
    private final AnalogInput        aimAbsoluteEncoder       = new AnalogInput(ArmConstants.AIM_ABSOLUTE_ENCODER_ANALOG_PORT);
    private final DigitalOutput      trapRelease              = new DigitalOutput(ArmConstants.TRAP_RELEASE_DIO_PORT);
    private double                   linkPivotSpeed           = 0;
    private double                   aimPivotSpeed            = 0;
    private boolean                  safetyEnabled            = false;
    private long                     safetyStartTime          = 0;
    private long                     trapReleaseStartTime     = 0;

    public ArmSubsystem() {

        linkMotor.setInverted(false);
        aimMotor.setInverted(true);

        linkMotor.getEncoder().setPosition(0);
        aimMotor.getEncoder().setPosition(0);

        double intakePositionConversionFactor  = 1; // todo: specify
        double intakeVelocityConversionFactor  = 1; // todo: specify
        double shooterPositionConversionFactor = 1; // todo: specify
        double shooterVelocityConversionFactor = 1; // todo: specify

        configureSparkMax(intakeMotor::restoreFactoryDefaults);
        configureSparkMax(intakeMotor::clearFaults);
        intakeEncoder = intakeMotor.getEncoder();
        configureSparkMax(() -> intakeEncoder.setPositionConversionFactor(intakePositionConversionFactor));
        configureSparkMax(() -> intakeEncoder.setVelocityConversionFactor(intakeVelocityConversionFactor));
        intakePid = intakeMotor.getPIDController();
        intakePid.setFeedbackDevice(intakeEncoder);
        configurePid(intakePid, 1, 0, 0, 0, 0);
        burnFlash(intakeMotor);

        configureSparkMax(shooterTopMotor::restoreFactoryDefaults);
        configureSparkMax(shooterTopMotor::clearFaults);
        shooterTopEncoder = shooterTopMotor.getEncoder();
        configureSparkMax(() -> shooterTopEncoder.setPositionConversionFactor(shooterPositionConversionFactor));
        configureSparkMax(() -> shooterTopEncoder.setVelocityConversionFactor(shooterVelocityConversionFactor));
        shooterTopPid = shooterTopMotor.getPIDController();
        shooterTopPid.setFeedbackDevice(shooterTopEncoder);
        configurePid(shooterTopPid, 1, 0, 0, 0, 0);
        burnFlash(shooterTopMotor);

        configureSparkMax(shooterBottomMotor::restoreFactoryDefaults);
        configureSparkMax(shooterBottomMotor::clearFaults);
        shooterBottomEncoder = shooterBottomMotor.getEncoder();
        configureSparkMax(() -> shooterBottomEncoder.setPositionConversionFactor(shooterPositionConversionFactor));
        configureSparkMax(() -> shooterBottomEncoder.setVelocityConversionFactor(shooterVelocityConversionFactor));
        shooterBottomPid = shooterBottomMotor.getPIDController();
        shooterBottomPid.setFeedbackDevice(shooterBottomEncoder);
        configurePid(shooterBottomPid, 1, 0, 0, 0, 0);
        burnFlash(shooterBottomMotor);

    }

    private void configurePid(SparkPIDController pid, double p, double i, double d, double ff, double iz) {
        configureSparkMax(() -> pid.setP(p, 0));
        configureSparkMax(() -> pid.setI(i, 0));
        configureSparkMax(() -> pid.setD(d, 0));
        configureSparkMax(() -> pid.setFF(ff, 0));
        configureSparkMax(() -> pid.setIZone(iz, 0));
        configureSparkMax(() -> pid.setOutputRange(-1, 1, 0));
        configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(false));
    }

    /**
     * Run the configuration until it succeeds or times out.
     *
     * @param config Lambda supplier returning the error state.
     */
    private void configureSparkMax(Supplier<REVLibError> config) {
        for (int i = 0; i < maxSparkMaxConfigRetries; i++) {
            if (config.get() == REVLibError.kOk) {
                return;
            }
        }
        DriverStation.reportWarning("Failure configuring motor ", true);
    }

    private void burnFlash(CANSparkMax motor) {
        try {
            Thread.sleep(200);
        }
        catch (Exception e) {
        }
        configureSparkMax(() -> motor.burnFlash());
    }

    private double getAimAbsoluteEncoderVoltage() {
        // 0-5V range = 0-360 deg
        return aimAbsoluteEncoder.getVoltage();
    }

    // todo: fixme: return rotation2d or specify unit in either javadoc or method name
    public double getAimAngle() {

        // The aim encoder should wrap at 4V.
        double aimEncoderVoltage = getAimAbsoluteEncoderVoltage();

        double motorEncoderValue = aimMotor.getEncoder().getPosition();

        // NOTE: the aim encoder can wrap at high angles (trap)
        // We need to add 5V if the encoder value is high.
        // It will not wrap twice
        // FIXME test these values
        if (motorEncoderValue > 100 && aimEncoderVoltage < 2.5) {
            aimEncoderVoltage += 5;
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
        double voltage = getLinkAbsoluteEncoderVoltage();
        if (voltage < 3) {
            voltage += 5;
        }
        // The conversion from volts to degrees
        double angle = voltage
            * ArmConstants.LINK_ABSOLUTE_ENCODER_DEG_PER_VOLT
            + ArmConstants.LINK_ABSOLUTE_ENCODER_OFFSET_DEG;

        // round to 2 decimal places
        return Math.round(angle * 100) / 100.0d;
    }

    /**
     * Return the bottom shooter encoder speed in RPM
     */
    public double getBottomShooterEncoderSpeed() {
        return shooterBottomEncoder.getVelocity();
    }

    /**
     * Return the top shooter encoder speed in RPM
     */
    public double getTopShooterEncoderSpeed() {
        return shooterTopEncoder.getVelocity();
    }

    /**
     * Return the intake encoder speed in RPM
     */
    public double getIntakeEncoderSpeed() {
        return Math.round(intakeEncoder.getVelocity() * 100) / 100.0;
    }

    public double getIntakePosition() {
        return intakeMotor.getEncoder().getPosition();
    }

    public boolean isLinkAtLowerLimit() {
        return !linkLowerLimitSwitch.get() || getLinkAngle() < ArmConstants.LINK_MIN_DEGREES;
    }

    public boolean isNoteDetected() {
        return noteDetector.get();
    }

    /**
     * Set the arm speeds FOR TEST MODE ONLY. Note this requires manual intervention.
     */
    public void setArmPivotTestSpeeds(double linkSpeedPct, double aimSpeedPct) {
        setLinkPivotSpeed(linkSpeedPct);
        setAimPivotSpeed(aimSpeedPct);
    }

    public void setLinkPivotSpeed(double speedPct) {
        this.linkPivotSpeed = speedPct;
    }

    public void setAimPivotSpeed(double speedPct) {
        this.aimPivotSpeed = speedPct;
    }

    /**
     * @deprecated use setIntakeRpm instead. This will set the RPM value to the provided number,
     * which will typically be less than 1 (which is very slow!)
     */
    public void setIntakeSpeed(double intakeSpeedPct) {
        setIntakeRpm(intakeSpeedPct);
    }

    /**
     * @deprecated use setShooterRpm instead. This will set the RPM value to the provided number,
     * which will typically be less than 1 (which is very slow!)
     */
    public void setShooterSpeed(double topShooterSpeedPct, double bottomShooterSpeedPct) {
        setShooterRpm(topShooterSpeedPct, bottomShooterSpeedPct);
    }

    public void setShooterRpm(double shooterRpm) {
        this.setShooterRpm(shooterRpm, shooterRpm);
    }

    public void setShooterRpm(double topShooterRpm, double bottomShooterRpm) {
        shooterTopPid.setReference(topShooterRpm, CANSparkMax.ControlType.kVelocity);
        shooterBottomPid.setReference(bottomShooterRpm, CANSparkMax.ControlType.kVelocity);
    }

    public void setIntakeRpm(double intakeRpm) {
        intakePid.setReference(intakeRpm, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * @deprecated use setShooterRpm instead. This will set the RPM value to the provided number,
     * which will typically be less than 1 (which is very slow!)
     */
    public void setShooterSpeed(double shooterSpeedPct) {
        this.setShooterSpeed(shooterSpeedPct, shooterSpeedPct);
    }

    public void stop() {
        setLinkPivotSpeed(0);
        setAimPivotSpeed(0);
        setIntakeRpm(0);
        setShooterRpm(0);
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
        checkArmSafety();

        if (!ArmConstants.DISABLE_LINK) {
            linkMotor.set(linkPivotSpeed);
        }

        if (!ArmConstants.DISABLE_AIM) {
            aimMotor.set(aimPivotSpeed);
        }

        // Latch the arm safety for 2 seconds when a safety condition
        // is activated.
        if (safetyEnabled) {
            if ((System.currentTimeMillis() - safetyStartTime) > 2000) {
                safetyEnabled = false;
            }
        }

        // Turn trap off after 200 millis
        if (trapReleased()) {
            if (System.currentTimeMillis() - trapReleaseStartTime >= 200) {
                trapRelease.set(false);
            }
        }

        /*
         * Update the SmartDashboard
         */
        Telemetry.arm.intakeSpeed                = intakeEncoder.getVelocity();
        Telemetry.arm.intakeEncoderSpeed         = getIntakeEncoderSpeed();
        Telemetry.arm.topShooterSpeed            = shooterTopEncoder.getVelocity();
        Telemetry.arm.bottomShooterSpeed         = shooterBottomEncoder.getVelocity();
        Telemetry.arm.topShooterEncoderSpeed     = getTopShooterEncoderSpeed();
        Telemetry.arm.bottomShooterEncoderSpeed  = getBottomShooterEncoderSpeed();
        Telemetry.arm.linkPivotSpeed             = linkPivotSpeed;
        Telemetry.arm.linkAngle                  = getLinkAngle();
        Telemetry.arm.linkAbsoluteEncoderVoltage = getLinkAbsoluteEncoderVoltage();
        Telemetry.arm.isLinkAtLowerLimit         = isLinkAtLowerLimit();
        Telemetry.arm.aimPivotSpeed              = aimPivotSpeed;
        Telemetry.arm.aimAngle                   = getAimAngle();
        Telemetry.arm.aimAbsoluteEncoderVoltage  = getAimAbsoluteEncoderVoltage();
        Telemetry.arm.noteDetected               = isNoteDetected();
        Telemetry.arm.safetyEnabled              = safetyEnabled;
        Telemetry.arm.trapReleased               = trapReleased();
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append("Link ").append(getLinkAngle()).append("deg (").append(linkPivotSpeed).append(") ")
            .append(isLinkAtLowerLimit() ? "LINK LOWER LIMIT" : "")
            .append("Aim ").append(getAimAngle()).append("deg (").append(aimPivotSpeed).append(") ")
            .append("Intake ")
            .append(String.format("%.2f", getIntakeEncoderSpeed())).append(' ')
            .append("TopShooter ")
            .append(String.format("%.2f", getTopShooterEncoderSpeed())).append(' ')
            .append("BottomShooter ")
            .append(String.format("%.2f", getBottomShooterEncoderSpeed())).append(' ')
            .append("Game Piece ").append(isNoteDetected());

        return sb.toString();
    }

    private void checkArmSafety() {

        if (DISABLE_ARM_SAFETY_MODE) {
            return;
        }

        double linkAngle  = getLinkAngle();
        double aimAngle   = getAimAngle();
        double totalAngle = linkAngle + aimAngle;


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
        if (linkPivotSpeed < 0 && (linkAngle <= ArmConstants.LINK_MIN_DEGREES
            || isLinkAtLowerLimit())) {
            linkPivotSpeed  = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * Never drive the link past 125 deg.
         *
         * The arm never needs to be that high.
         */
        if (linkPivotSpeed > 0 && linkAngle >= ArmConstants.LINK_MAX_DEGREES) {
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
        if (aimPivotSpeed > 0 && aimAngle >= ArmConstants.AIM_MAX_DEGREES) {
            aimPivotSpeed   = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * Never drive the aim angle < 60 deg.
         *
         * The aim never needs to be that low.
         */
        if (aimPivotSpeed < 0 && aimAngle <= ArmConstants.AIM_MIN_DEGREES) {
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
        if (totalAngle <= ArmConstants.ARM_MIN_ANGLE_SUM) {
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

        if (totalAngle >= ArmConstants.ARM_MAX_ANGLE_SUM) {
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

        /*
         * Compact Pose
         * 
         * When getting close to the compact pose and descending into compact pose, slow down the
         * motors to avoid slamming.
         */
        double  linkCompactDelta    = linkAngle - COMPACT_ARM_POSITION.linkAngle;
        double  absLinkCompactDelta = Math.abs(linkCompactDelta);
        boolean linkClose           = absLinkCompactDelta < COMPACT_LINK_SLOW_RANGE_DEG;
        double  aimCompactDelta     = aimAngle - COMPACT_ARM_POSITION.aimAngle;
        double  absAimCompactDelta  = Math.abs(aimCompactDelta);
        boolean aimClose            = absAimCompactDelta < COMPACT_AIM_SLOW_RANGE_DEG;
        if (linkClose && aimClose) {
            if (aimPivotSpeed < 0 && Math.abs(aimPivotSpeed) > SLOW_AIM_SPEED) {
                aimPivotSpeed = -SAFE_AIM_SPEED;
//                log(String.format("Compacting - aim safety mode link: %.2f aim: %.2f total: %.2f", linkAngle, aimAngle,
//                    totalAngle));
            }
            if (linkPivotSpeed < 0 && Math.abs(linkPivotSpeed) > SLOW_LINK_SPEED) {
                linkPivotSpeed = -SLOW_LINK_SPEED;
//                log(String.format("Compacting - link safety mode link: %.2f aim: %.2f total: %.2f", linkAngle, aimAngle,
//                    totalAngle));
            }
        }
    }

    public double getShooterPosition() {
        return shooterTopMotor.getEncoder().getPosition();
    }

    private boolean trapReleased() {
        return trapRelease.get();
    }

    public void releaseTrap() {
        // TODO: fixme: write trap release code

        if (trapReleased()) {
            return;
        }

        trapRelease.set(true);
        trapReleaseStartTime = System.currentTimeMillis();
    }

}