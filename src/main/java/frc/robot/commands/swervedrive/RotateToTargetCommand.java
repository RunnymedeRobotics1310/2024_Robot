package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.BotTarget;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class RotateToTargetCommand extends BaseDriveCommand {

    private final BotTarget blueTarget;
    private final BotTarget redTarget;
    private BotTarget       target;
    private final boolean   forwards;
    int                     alignedCount = 0;


    public static RotateToTargetCommand createRotateToSpeakerCommand(SwerveSubsystem swerve) {
        return new RotateToTargetCommand(swerve, BotTarget.BLUE_SPEAKER, BotTarget.RED_SPEAKER);
    }

    public static RotateToTargetCommand createRotateToSourceCommand(SwerveSubsystem swerve) {
        return new RotateToTargetCommand(swerve, BotTarget.BLUE_SOURCE, BotTarget.RED_SOURCE);
    }

    /**
     * Turn the robot to face the vision target specified
     * 
     * @param swerve the swerve drive subsystem
     */
    private RotateToTargetCommand(SwerveSubsystem swerve, BotTarget blueTarget, BotTarget redTarget) {
        super(swerve);
        this.blueTarget = blueTarget;
        this.redTarget  = redTarget;
        this.target     = null;
        this.forwards   = getForwards(blueTarget);
    }

    private boolean getForwards(BotTarget tgt) {

        if (tgt == BotTarget.BLUE_SPEAKER || tgt == BotTarget.BLUE_AMP) {
            return false;
        }
        else {
            return true;
        }
    }

    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            target = blueTarget;
        }
        else {
            target = redTarget;
        }
        logCommandStart("Target: " + target);
        alignedCount = 0;
    }


    @Override
    public void execute() {
        super.execute();

        Rotation2d heading    = super.getHeadingToFieldPosition(target.getLocation().toTranslation2d())
            .plus(Rotation2d.fromDegrees(180 * (forwards ? 0 : 1)));
        Pose2d     targetPose = new Pose2d(swerve.getPose().getTranslation(), heading);
        driveToFieldPose(targetPose, MAX_TRANSLATION_SPEED_MPS);
    }


    @Override
    public boolean isFinished() {
        if (isAligned()) {
            alignedCount++;
        }
        else {
            alignedCount = 0;
        }
        return alignedCount >= 10;
    }


    private boolean isAligned() {
        Rotation2d heading = super.getHeadingToFieldPosition(target.getLocation().toTranslation2d())
            .plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));
        return isCloseEnough(heading);
    }
}