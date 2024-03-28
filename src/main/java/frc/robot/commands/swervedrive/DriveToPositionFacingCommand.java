package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.RunnymedeUtils.format;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class DriveToPositionFacingCommand extends BaseDriveCommand {

    private final Translation2d bluePositionToDriveToward;
    private final Translation2d bluePositionToFace;
    private final Translation2d redPositionToDriveToward;
    private final Translation2d redPositionToFace;
    private final double        maxSpeedMPS;

    private Translation2d       positionToDriveToward;
    private Translation2d       positionToFace;

    /**
     * Drive to the specified location while facing that location at the same time (i.e. aim for
     * robot-relative heading of 0 and field-relative heading is not important).
     *
     * Note, that it's not possible to know which direction to face when the robot arrives at the
     * target location, so heading error is not considered to be a factor in the completion of this
     * command.
     */
    public DriveToPositionFacingCommand(
        SwerveSubsystem swerve, Translation2d bluePosition, Translation2d redPosition, double maxSpeedMPS) {
        super(swerve);
        this.bluePositionToDriveToward = bluePosition;
        this.bluePositionToFace        = bluePosition;
        this.redPositionToDriveToward  = redPosition;
        this.redPositionToFace         = redPosition;
        this.maxSpeedMPS               = maxSpeedMPS;
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
        return isCloseEnough(positionToDriveToward);
    }
}