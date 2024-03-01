package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private final LightsSubsystem lightsSubsystem;

    private double                linkPivotSpeed      = 0;
    private double                aimPivotSpeed       = 0;
    private double                intakeSpeed         = 0;
    private double                shooterSpeed        = 0;

    private double                intakeSpeedEncoder  = 0;
    private double                shooterSpeedEncoder = 0;
    private double                linkAngleEncoder    = 0;
    private double                aimAngleEncoder     = 0;

    private DigitalInput          noteDetector        = new DigitalInput(1);

    private boolean               safetyEnabled       = false;
    private long                  safetyStartTime     = 0;

    public ArmSubsystem(LightsSubsystem lightsSubsystem) {

        this.lightsSubsystem  = lightsSubsystem;

        // This is faking an angle encoder
        this.aimAngleEncoder  = ArmConstants.COMPACT_ARM_POSITION.aimAngle;
        this.linkAngleEncoder = ArmConstants.COMPACT_ARM_POSITION.linkAngle;
    }

    public double getAimAngle() {
        return aimAngleEncoder;
    }

    public double getLinkAngle() {
        return linkAngleEncoder;
    }

    public double getShooterSpeed() {
        return shooterSpeedEncoder;
    }

    public double getIntakeSpeed() {
        return intakeSpeedEncoder;
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
    }

    public void setShooterSpeed(double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
    }

    public void stop() {
        setLinkPivotSpeed(0);
        setAimPivotSpeed(0);
        setIntakeSpeed(0);
        setShooterSpeed(0);
        intakeSpeedEncoder  = 0;
        shooterSpeedEncoder = 0;
    }

    @Override
    public void periodic() {

        // TODO REMOVE this code when the real robot is available

        // FAKE THE MOTOR ACTION

        // Fake some momentum in the motors by having them
        // slowly ramp towards the set speed. In reality,
        // this will happen much faster.
        if (intakeSpeed > intakeSpeedEncoder) {
            intakeSpeedEncoder += .002;
        }
        else if (intakeSpeed < intakeSpeedEncoder) {
            intakeSpeedEncoder -= .002;
        }

        if (shooterSpeed > shooterSpeedEncoder) {
            shooterSpeedEncoder += .002;
        }
        else if (shooterSpeed < shooterSpeedEncoder) {
            shooterSpeedEncoder -= .002;
        }

        // Move the aim and link angles based on the speed.
        aimAngleEncoder  = Math.min(200, Math.max(0, aimAngleEncoder + aimPivotSpeed));
        linkAngleEncoder = Math.min(200, Math.max(0, linkAngleEncoder + linkPivotSpeed));

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

        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        SmartDashboard.putNumber("Encoder Intake Speed", intakeSpeedEncoder);

        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putNumber("Encoder Shooter Speed", shooterSpeedEncoder);

        SmartDashboard.putNumber("Link Speed", linkPivotSpeed);
        SmartDashboard.putNumber("Link Angle", getLinkAngle());

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
            .append("Intake ").append(intakeSpeed).append(", ").append(intakeSpeedEncoder).append(' ')
            .append("Shooter ").append(shooterSpeed).append(", ").append(shooterSpeedEncoder).append(' ')
            .append("Game Piece ").append(isNoteDetected());

        return sb.toString();
    }

    /*
     * Update the lights based on the current state of the arm
     * FIXME: the lighting pattern is specific to Shifty.
     */
    private void updateLights() {

        boolean intakeAtTargetSpeed = Math.abs(shooterSpeed - intakeSpeedEncoder) < .05;

        lightsSubsystem.setIntakeSpeed(intakeSpeedEncoder, intakeAtTargetSpeed);

        boolean shooterAtTargetSpeed = Math.abs(shooterSpeed - shooterSpeedEncoder) < .05;

        lightsSubsystem.setShooterSpeed(shooterSpeedEncoder, shooterAtTargetSpeed);

        lightsSubsystem.setLinkAngle(getLinkAngle());

        lightsSubsystem.setAimAngle(getAimAngle());

        lightsSubsystem.setNoteHeld(isNoteDetected());
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
        if (linkPivotSpeed < 0 && linkAngleEncoder <= ArmConstants.LINK_MIN) {
            linkPivotSpeed  = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * Never drive the link past 125 deg.
         *
         * The arm never needs to be that high.
         */
        if (linkPivotSpeed > 0 && linkAngleEncoder >= ArmConstants.LINK_MAX) {
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
        if (aimPivotSpeed > 0 && aimAngleEncoder >= ArmConstants.AIM_MAX) {
            aimPivotSpeed   = 0;
            safetyEnabled   = true;
            safetyStartTime = System.currentTimeMillis();
        }

        /*
         * Never drive the aim angle < 60 deg.
         *
         * The aim never needs to be that low.
         */
        if (aimPivotSpeed < 0 && aimAngleEncoder <= ArmConstants.AIM_MIN) {
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
        if (aimAngleEncoder + linkAngleEncoder <= ArmConstants.MIN_ANGLE_SUM) {
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

        if (aimAngleEncoder + linkAngleEncoder >= ArmConstants.MAX_ANGLE_SUM) {
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
