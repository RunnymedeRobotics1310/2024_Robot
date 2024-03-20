package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;

public class DriveRobotOrientedCommand extends BaseDriveCommand {

    private final Translation2d robotRelativeTranslation;
    private final Rotation2d    robotRelativeHeading;
    private Pose2d              desiredPose = null;
    private double speed = 0;

    /**
     * Drive at speed given in the direction specified at the heading specified
     * 
     * @param swerve drive subsystem
     * @param robotRelativeTranslation a translation relative to the robot
     * @param robotRelativeHeading the FIELD-ORIENTED heading to face
     * @param speed the speed to drive at
     */
    public DriveRobotOrientedCommand(SwerveSubsystem swerve, Translation2d robotRelativeTranslation,
        Rotation2d robotRelativeHeading, double speed) {
        super(swerve);
        this.robotRelativeTranslation = robotRelativeTranslation;
        this.robotRelativeHeading     = robotRelativeHeading;
        this.speed = speed;
    }

    /**
     * Drive as fast as possible in the direction specified at the heading specified
     *
     * @param swerve drive subsystem
     * @param robotRelativeTranslation a translation relative to the robot
     * @param robotRelativeHeading the FIELD-ORIENTED heading to face
     */
    public DriveRobotOrientedCommand(SwerveSubsystem swerve, Translation2d robotRelativeTranslation,
                                     Rotation2d robotRelativeHeading) {
        this(swerve, robotRelativeTranslation, robotRelativeHeading, MAX_TRANSLATION_SPEED_MPS);
    }

    @Override
    public void initialize() {
        logCommandStart(
            "robotRelativeTranslation: " + robotRelativeTranslation + " robotRelativeHeading: " + robotRelativeHeading);
        Rotation2d    desiredFieldOrientedHeading  = swerve.getPose().getRotation().plus(robotRelativeHeading);
        Translation2d desiredFieldOrientedPosition = swerve.getPose().getTranslation().plus(robotRelativeTranslation);
        this.desiredPose = new Pose2d(desiredFieldOrientedPosition, desiredFieldOrientedHeading);
    }

    @Override
    public void execute() {
        super.execute();
        driveToFieldPose(desiredPose, speed);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return isCloseEnough(desiredPose.getTranslation()) && isCloseEnough(desiredPose.getRotation());
    }
}
