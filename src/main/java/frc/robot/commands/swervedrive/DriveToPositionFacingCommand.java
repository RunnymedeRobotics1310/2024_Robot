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

    private Translation2d       positionToDriveToward;
    private Translation2d       positionToFace;
    private double              maxSpeedMPS = MAX_TRANSLATION_SPEED_MPS;

    /**
     * Drive as fast as possible to the specified location while facing another specified position.
     * Useful while driving one way and locking on another target.
     */
    public DriveToPositionFacingCommand(SwerveSubsystem swerve, Translation2d bluePositionToDriveToward,
        Translation2d bluePositionToFace, Translation2d redPositionToDriveToward, Translation2d redPositionToFace,
        double maxSpeedMPS) {
        super(swerve);
        this.redPositionToDriveToward  = redPositionToDriveToward;
        this.redPositionToFace         = redPositionToFace;
        this.bluePositionToDriveToward = bluePositionToDriveToward;
        this.bluePositionToFace        = bluePositionToFace;
        this.maxSpeedMPS               = maxSpeedMPS;
    }

    /**
     * Drive as fast as possible to the specified location while facing another specified position.
     * Useful while driving one way and locking on another target.
     */
    public DriveToPositionFacingCommand(SwerveSubsystem swerve, Translation2d bluePositionToDriveToward,
        Translation2d bluePositionToFace, Translation2d redPositionToDriveToward, Translation2d redPositionToFace) {
        super(swerve);
        this.redPositionToDriveToward  = redPositionToDriveToward;
        this.redPositionToFace         = redPositionToFace;
        this.bluePositionToDriveToward = bluePositionToDriveToward;
        this.bluePositionToFace        = bluePositionToFace;
    }

    /**
     * Drive as fast as possible to the specified location while facing another specified position.
     * Useful while driving one way and locking on another target.
     */
    public DriveToPositionFacingCommand(SwerveSubsystem swerve,
        Translation2d bluePositionToDriveAndFaceToward, Translation2d redPositionToDriveAndFaceToward, double maxSpeedMPS) {
        this(swerve, bluePositionToDriveAndFaceToward, bluePositionToDriveAndFaceToward,
            redPositionToDriveAndFaceToward, redPositionToDriveAndFaceToward, maxSpeedMPS);
    }

    /**
     * Drive as fast as possible to the specified location while facing another specified position.
     * Useful while driving one way and locking on another target.
     */
    public DriveToPositionFacingCommand(SwerveSubsystem swerve,
        Translation2d bluePositionToDriveAndFaceToward, Translation2d redPositionToDriveAndFaceToward) {
        this(swerve, bluePositionToDriveAndFaceToward, bluePositionToDriveAndFaceToward,
            redPositionToDriveAndFaceToward, redPositionToDriveAndFaceToward);
    }

    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            positionToDriveToward = bluePositionToDriveToward;
            positionToFace        = bluePositionToFace;
        }
        else {
            positionToDriveToward = redPositionToDriveToward;
            positionToFace        = redPositionToFace;
        }

        logCommandStart("drive: " + format(positionToDriveToward) + " face: " + format(positionToFace));
    }

    @Override
    public void execute() {
        super.execute();
        Pose2d     current  = swerve.getPose();
        Rotation2d heading  = positionToFace.minus(current.getTranslation()).getAngle();
        Pose2d     nextPose = new Pose2d(positionToDriveToward, heading);
        driveToFieldPose(nextPose, maxSpeedMPS);
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