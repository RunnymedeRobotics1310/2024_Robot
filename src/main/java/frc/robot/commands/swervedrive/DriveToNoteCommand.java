package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.commands.arm.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class DriveToNoteCommand extends BaseDriveCommand {
    private final JackmanVisionSubsystem jackman;
    private final ArmSubsystem arm;
    private long intakeStartTime;
    private boolean startTimeSet = false;
    private double speed;

    // todo: fixme: specify unit in speed param name (e.g. speedRPM, speedDegPerSec, etc.)
    public DriveToNoteCommand(SwerveSubsystem drive, ArmSubsystem arm, JackmanVisionSubsystem jackman, double speed) {

        super(drive);
        this.jackman = jackman;
        this.speed   = speed;
        this.arm     = arm;
        addRequirements(jackman);
    }

    public void execute() {
        super.execute();

        Rotation2d robotRelativeOffset = jackman.getNoteOffset();

        if (robotRelativeOffset != null) {

            if (Math.abs(robotRelativeOffset.getDegrees()) > 10) {
                speed = 0;
            }

            Rotation2d omega = computeOmega(robotRelativeOffset);
            swerve.driveRobotOriented(new ChassisSpeeds(speed, 0, omega.getRadians()));
        }

    }

    @Override
    public boolean isFinished() {
        Rotation2d robotRelativeOffset = jackman.getNoteOffset();

        if (arm.getIntakeEncoderSpeed() >= 10 && !startTimeSet) {
            intakeStartTime = System.currentTimeMillis();
            startTimeSet = true;
        }

        // return when note is detected
        if (System.currentTimeMillis() - intakeStartTime >= Constants.ArmConstants.INTAKE_SPINUP_WINDOW) {
            if (arm.getIntakeEncoderSpeed() <= 400 || arm.isNoteDetected()) {
                return true;
            }
        }
        return false;
    }
}

