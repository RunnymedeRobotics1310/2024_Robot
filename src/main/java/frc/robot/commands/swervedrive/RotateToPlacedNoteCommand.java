package frc.robot.commands.swervedrive;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RotateToPlacedNoteCommand extends BaseDriveCommand {

    private final Constants.BotTarget blueTarget;
    private final Constants.BotTarget redTarget;
    private Constants.BotTarget       target = null;

    /**
     * Turn the robot to face the vision target specified
     *
     * @param swerve the swerve drive subsystem
     */
    public RotateToPlacedNoteCommand(SwerveSubsystem swerve, Constants.BotTarget blueTarget, Constants.BotTarget redTarget) {
        super(swerve);
        this.blueTarget = blueTarget;
        this.redTarget  = redTarget;

    }

    @Override
    public void initialize() {
        this.target = RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue ? blueTarget : redTarget;
        logCommandStart("Target: " + target);
    }

    @Override
    public void execute() {
        super.execute();

        Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
        Rotation2d omega = computeOmega(delta);
        swerve.driveFieldOriented(new Translation2d(), omega);

    }

    @Override
    public boolean isFinished() {

        Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
        return isCloseEnough(delta);

    }

}