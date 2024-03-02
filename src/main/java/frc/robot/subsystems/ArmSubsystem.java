package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax     intakeMotor          = new CANSparkMax(ArmConstants.INTAKE_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax     shooterBottomMotor   = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS,
        MotorType.kBrushless);
    private final CANSparkMax     shooterTopMotor      = new CANSparkMax(ArmConstants.SHOOTER_MOTOR_CAN_ADDRESS + 1,
        MotorType.kBrushless);

    private DigitalInput          noteDetector         = new DigitalInput(ArmConstants.INTAKE_NOTE_DETECTOR_DIO_PORT);

    private double                intakeSpeed          = 0;
    private double                shooterSpeed         = 0;

    public double getShooterEncoderSpeed() {
        return shooterBottomMotor.getEncoder().getVelocity();
    }

    public double getIntakeEncoderSpeed() {
        return Math.round(intakeMotor.getEncoder().getVelocity() * 100) / 100.0;
    }

    public boolean isNoteDetected() {
        return noteDetector.get();
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
        setIntakeSpeed(0);
        setShooterSpeed(0);
    }

    @Override
    public void periodic() {

        /*
         * Update the SmartDashboard
         */

        SmartDashboard.putNumber("Intake Motor", intakeSpeed);
        SmartDashboard.putNumber("Intake Encoder Speed", getIntakeEncoderSpeed());

        SmartDashboard.putNumber("Shooter Motor", shooterSpeed);
        SmartDashboard.putNumber("Shooter Encoder Speed", getShooterEncoderSpeed());

        SmartDashboard.putBoolean("Note Detected", isNoteDetected());

    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append("Intake ").append(intakeSpeed).append(", ").append(getIntakeEncoderSpeed()).append(' ')
            .append("Shooter ").append(shooterSpeed).append(", ").append(getShooterEncoderSpeed()).append(' ')
            .append("Game Piece ").append(isNoteDetected());

        return sb.toString();
    }

}
