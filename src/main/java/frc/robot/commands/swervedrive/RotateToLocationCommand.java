package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class RotateToLocationCommand extends BaseDriveCommand {

    private final Translation2d blueTarget;
    private final Translation2d redTarget;
    private Translation2d       target = null;

    /**
     * Turn the robot to face the vision target specified
     *
     * @param swerve the swerve drive subsystem
     */
    public RotateToLocationCommand(SwerveSubsystem swerve, Translation2d blueTarget, Translation2d redTarget) {
        super(swerve);
        this.blueTarget = blueTarget;
        this.redTarget  = redTarget;
    }

    @Override
    public void initialize() {
        super.initialize();
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            target = blueTarget;
        }
        else {
            target = redTarget;
        }
    }

    @Override
    public void execute() {
        super.execute();
        Rotation2d delta = getHeadingToFieldPosition(target);
        Rotation2d omega = computeOmega(delta);
        swerve.driveFieldOriented(new Translation2d(), omega);
    }

    @Override
    public boolean isFinished() {
        Rotation2d delta = getHeadingToFieldPosition(target);
        return isCloseEnough(delta);
    }
}