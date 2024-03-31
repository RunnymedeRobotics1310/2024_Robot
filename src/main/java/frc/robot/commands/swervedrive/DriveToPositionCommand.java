package frc.robot.commands.swervedrive;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToPositionCommand extends BaseDriveCommand {

    private final Translation2d blueLocation;
    private final Translation2d redLocation;
    private final Pose2d        bluePose;
    private final Pose2d        redPose;

    private Pose2d              desiredPose;

    private double              speed;

    /**
     * Drive to the specified location, maintaining the current heading.
     */
    public DriveToPositionCommand(SwerveSubsystem swerve, Translation2d blueLocation, Translation2d redLocation) {
        super(swerve);
        this.bluePose     = null;
        this.redPose      = null;
        this.blueLocation = blueLocation;
        this.redLocation  = redLocation;
        this.desiredPose  = null;
        this.speed        = Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
    }

    /**
     * Drive as fast as possible to the specified pose.
     */
    public DriveToPositionCommand(SwerveSubsystem swerve, Pose2d bluePose, Pose2d redPose) {
        this(swerve, bluePose, redPose, Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS);
    }

    /**
     * Drive at the specified speed to the specified pose.
     */
    public DriveToPositionCommand(SwerveSubsystem swerve, Pose2d bluePose, Pose2d redPose, double speed) {
        super(swerve);
        this.bluePose     = bluePose;
        this.redPose      = redPose;
        this.blueLocation = null;
        this.redLocation  = null;
        this.desiredPose  = null;
        this.speed        = speed;
    }

    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            if (bluePose == null) {
                desiredPose = new Pose2d(blueLocation, swerve.getPose().getRotation());
            }
            else {
                desiredPose = bluePose;
            }
        }
        else {
            if (redPose == null) {
                desiredPose = new Pose2d(redLocation, swerve.getPose().getRotation());
            }
            else {
                desiredPose = redPose;
            }
        }
        logCommandStart("desiredPose: " + desiredPose);
    }

    @Override
    public void execute() {
        super.execute();
        swerve.driveToFieldPose(desiredPose, speed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return swerve.isCloseEnough(desiredPose);
    }
}