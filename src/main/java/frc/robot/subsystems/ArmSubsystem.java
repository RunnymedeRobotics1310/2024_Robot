package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.ArmConstants;
import frc.robot.telemetry.Telemetry;

import static frc.robot.Constants.ArmConstants.DISABLE_ARM_SAFETY_MODE;


public class ArmSubsystem extends RunnymedeSubsystemBase {
    private final CANSparkMax   linkMotor            = new CANSparkMax(ArmConstants.LINK_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax   aimMotor             = new CANSparkMax(ArmConstants.AIM_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax   intakeMotor          = new CANSparkMax(ArmConstants.INTAKE_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax   shooterBottomMotor   = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax   shooterTopMotor      = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS + 1,
        MotorType.kBrushless);
    private final DigitalInput  linkLowerLimitSwitch = new DigitalInput(ArmConstants.LINK_LOWER_LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput  noteDetector         = new DigitalInput(ArmConstants.INTAKE_NOTE_DETECTOR_DIO_PORT);
    private final AnalogInput   linkAbsoluteEncoder  = new AnalogInput(ArmConstants.LINK_ABSOLUTE_ENCODER_ANALOG_PORT);
    private final AnalogInput   aimAbsoluteEncoder   = new AnalogInput(ArmConstants.AIM_ABSOLUTE_ENCODER_ANALOG_PORT);
    private final DigitalOutput trapRelease          = new DigitalOutput(ArmConstants.TRAP_RELEASE_DIO_PORT);
    private double              linkPivotSpeed       = 0;
    private double              aimPivotSpeed        = 0;
    private double              intakeSpeed          = 0;
    private double              topShooterSpeed      = 0;
    private double              bottomShooterSpeed   = 0;
    private boolean             safetyEnabled        = false;
    private long                safetyStartTime      = 0;
    private long                trapReleaseStartTime = 0;

    public ArmSubsystem() {

        linkMotor.setInverted(false);
        aimMotor.setInverted(true);

        linkMotor.getEncoder().setPosition(0);
        aimMotor.getEncoder().setPosition(0);

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

    // todo: fixme: specify unit in either javadoc or method name
    public double getBottomShooterEncoderSpeed() {
        return shooterBottomMotor.getEncoder().getVelocity();
    }

    // todo: fixme: specify unit in either javadoc or method name
    public double getTopShooterEncoderSpeed() {
        return shooterTopMotor.getEncoder().getVelocity();
    }

    // todo: fixme: specify unit in either javadoc or method name
    public double getIntakeEncoderSpeed() {
        return Math.round(intakeMotor.getEncoder().getVelocity() * 100) / 100.0;
    }

    public double getIntakePosition() {
        return intakeMotor.getEncoder().getPosition();
    }

    // todo: fixme: specify unit in either javadoc or method name
    // changed to use static link pose and aim from link pose
    public Translation2d getShooterXY() {

        // calculate angle of bar
        double       aimMotorAngle = getAimAngle() + 48;

        final double hypM          = 0.20955;

        double       yDifference;
        double       xDifference;
        double       shooterX;
        double       shooterY;

        if (aimMotorAngle > 90) {
            aimMotorAngle -= 90;
            yDifference    = hypM * (Math.cos(aimMotorAngle));
            xDifference    = hypM * (Math.sin(aimMotorAngle));
            shooterX       = ArmConstants.AIM_X_SHOOTING - xDifference;
            shooterY       = ArmConstants.AIM_Y_SHOOTING + yDifference;
        }
        else if (aimMotorAngle < 90) {
            aimMotorAngle += 90;
            yDifference    = hypM * (Math.cos(aimMotorAngle));
            xDifference    = hypM * (Math.sin(aimMotorAngle));
            shooterX       = ArmConstants.AIM_X_SHOOTING - xDifference;
            shooterY       = ArmConstants.AIM_Y_SHOOTING + yDifference;

        }
        // aimMotorAngle == 90
        else {
            shooterX = 0.20955;
            shooterY = 0;
        }

        return new Translation2d(shooterX, shooterY);
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

    public void setIntakeSpeed(double intakeSpeedPct) {
        this.intakeSpeed = intakeSpeedPct;
    }

    public void setShooterSpeed(double topShooterSpeedPct, double bottomShooterSpeedPct) {
        this.topShooterSpeed    = topShooterSpeedPct;
        this.bottomShooterSpeed = bottomShooterSpeedPct;
    }

    public void setShooterSpeed(double shooterSpeedPct) {
        this.setShooterSpeed(shooterSpeedPct, shooterSpeedPct);
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
        checkArmSafety();

        if (!ArmConstants.DISABLE_LINK) {
            linkMotor.set(linkPivotSpeed);
        }

        if (!ArmConstants.DISABLE_AIM) {
            aimMotor.set(aimPivotSpeed);
        }
        intakeMotor.set(intakeSpeed);
        shooterTopMotor.set(topShooterSpeed);
        shooterBottomMotor.set(bottomShooterSpeed);

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
        Telemetry.arm.intakeSpeed                = intakeSpeed;
        Telemetry.arm.intakeEncoderSpeed         = getIntakeEncoderSpeed();
        Telemetry.arm.topShooterSpeed            = topShooterSpeed;
        Telemetry.arm.bottomShooterSpeed         = bottomShooterSpeed;
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
            .append("Intake ").append(intakeSpeed).append(", ").append(getIntakeEncoderSpeed()).append(' ')
            .append("TopShooter ").append(topShooterSpeed).append(", ").append(getBottomShooterEncoderSpeed()).append(' ')
            .append("BottomShooter ").append(bottomShooterSpeed).append(", ").append(getBottomShooterEncoderSpeed()).append(' ')
            .append("Game Piece ").append(isNoteDetected());

        return sb.toString();
    }

    private void checkArmSafety() {

        if (DISABLE_ARM_SAFETY_MODE) {
            return;
        }

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
        if (linkPivotSpeed < 0 && (getLinkAngle() <= ArmConstants.LINK_MIN_DEGREES
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