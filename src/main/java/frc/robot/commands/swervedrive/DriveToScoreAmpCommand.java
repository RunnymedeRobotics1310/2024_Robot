package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class DriveToScoreAmpCommand extends BaseDriveCommand {

    private final Pose2d  bluePose;
    private final Pose2d  redPose;

    private Translation2d location;
    private Translation2d nearby;
    private Rotation2d    heading;
    private double        speed;

    private enum State {
        MOVE_NEARBY, ROTATING, CLOSE_ALIGNING, DONE
    }

    private State state = State.MOVE_NEARBY;

    public DriveToScoreAmpCommand(SwerveSubsystem swerve) {
        super(swerve);
        this.bluePose = Constants.UsefulPoses.SCORE_BLUE_AMP;
        this.redPose  = Constants.UsefulPoses.SCORE_RED_AMP;
        this.heading  = null;
        this.speed    = Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
    }

    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            location = bluePose.getTranslation();
            heading  = bluePose.getRotation();
        }
        else {
            location = redPose.getTranslation();
            heading  = redPose.getRotation();
        }
        nearby = location.minus(new Translation2d(0, .75));
        logCommandStart("desiredPose: " + new Pose2d(location, heading));
        if (isCloseEnough(nearby)) {
            state = State.ROTATING;
        }
        else {
            state = State.MOVE_NEARBY;
        }
    }

    @Override
    public void execute() {
        super.execute();

        switch (state) {
        case MOVE_NEARBY:
            driveToFieldPose(new Pose2d(nearby, heading), speed);
            if (isCloseEnough(nearby)) {
                state = State.ROTATING;
            }
            break;
        case ROTATING:
            driveToFieldPose(new Pose2d(nearby, heading), speed);
            if (isCloseEnough(heading)) {
                state = State.CLOSE_ALIGNING;
            }
            break;
        case CLOSE_ALIGNING:
            driveToFieldPose(new Pose2d(location, heading), speed / 2);
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return isCloseEnough(location) && isCloseEnough(heading);
    }
}