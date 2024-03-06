package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.yagsl.YagslSubsystem;


public class DriveToNoteCommand extends BaseDriveCommand {
    private final JackmanVisionSubsystem jackman;
    private final ArmSubsystem armSubsystem;
    private Pose2d initialPose;
    private Rotation2d robotRelativeTranslation;
    private double speed;
    public DriveToNoteCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, JackmanVisionSubsystem jackman, double speed) {

        super(swerve);
        this.armSubsystem = armSubsystem;
        this.jackman = jackman;
        this.speed = speed;
        addRequirements(jackman);
    }

    @Override
    public void initialize() {
        this.initialPose = swerve.getPose();
    }

    public void execute() {
        super.execute();

        robotRelativeTranslation = jackman.getNoteOffset();

        if (robotRelativeTranslation != null) {

            if (Math.abs(robotRelativeTranslation.getDegrees()) > 10) {
                speed = 0;
            }

            Rotation2d omega = computeOmega(robotRelativeTranslation);
            swerve.driveRobotOriented(new ChassisSpeeds(speed, 0, omega.getRadians()));
        }

    }

    @Override
    public boolean isFinished() {
        // IMPORTANT: Only ever used in deadline with intake command
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
