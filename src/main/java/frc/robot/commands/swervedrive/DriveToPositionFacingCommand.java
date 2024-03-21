package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
import static frc.robot.RunnymedeUtils.format;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class DriveToPositionFacingCommand extends BaseDriveCommand {


    private final Translation2d redPositionToDriveToward;
    private final Translation2d redPositionToFace;
    private final Translation2d bluePositionToDriveToward;
    private final Translation2d bluePositionToFace;

    private Translation2d positionToDriveToward;
    private Translation2d positionToFace;

    /**
     * Drive as fast as possible to the specified location while facing another specified position.
     * Useful while driving one way and locking on another target.
     */
    public DriveToPositionFacingCommand(SwerveSubsystem swerve, Translation2d redPositionToDriveToward,
        Translation2d redPositionToFace, Translation2d bluePositionToDriveToward, Translation2d bluePositionToFace) {
        super(swerve);
        this.redPositionToDriveToward = redPositionToDriveToward;
        this.redPositionToFace = redPositionToFace;
        this.bluePositionToDriveToward = bluePositionToDriveToward;
        this.bluePositionToFace = bluePositionToFace;
    }

    /**
     * Drive as fast as possible to the specified location while facing another specified position.
     * Useful while driving one way and locking on another target.
     */
    public DriveToPositionFacingCommand(SwerveSubsystem swerve, Translation2d redPositionToDriveAndFaceToward,
                                        Translation2d bluePositionToDriveAndFaceToward) {
        this(swerve, redPositionToDriveAndFaceToward, redPositionToDriveAndFaceToward, bluePositionToDriveAndFaceToward, bluePositionToDriveAndFaceToward);
    }

    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            positionToDriveToward = bluePositionToDriveToward;
            positionToFace = bluePositionToFace;
        } else {
            positionToDriveToward = redPositionToDriveToward;
            positionToFace = redPositionToFace;
        }

        logCommandStart("drive: " + format(positionToDriveToward) + " face: " + format(positionToFace));
    }

    @Override
    public void execute() {
        super.execute();
        Pose2d     current  = swerve.getPose();
        Rotation2d heading  = positionToFace.minus(current.getTranslation()).getAngle();
        Pose2d     nextPose = new Pose2d(positionToDriveToward, heading);
        driveToFieldPose(nextPose, MAX_TRANSLATION_SPEED_MPS);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        Rotation2d heading = getHeadingToFieldPosition(positionToFace);
        return isCloseEnough(positionToDriveToward) && isCloseEnough(heading);
    }
}