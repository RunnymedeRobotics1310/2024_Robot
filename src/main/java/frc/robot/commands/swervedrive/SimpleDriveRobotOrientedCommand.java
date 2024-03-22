package frc.robot.commands.swervedrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SimpleDriveRobotOrientedCommand extends BaseDriveCommand {
    private final double vX;
    private final double vY;
    private final double omega;
    private final double timeout;


    public SimpleDriveRobotOrientedCommand(SwerveSubsystem swerve, double xSpeedMps, double ySpeedMps, double omegaRadPerSec,
        double timeout) {
        super(swerve);
        this.vX      = xSpeedMps;
        this.vY      = ySpeedMps;
        this.omega   = omegaRadPerSec;
        this.timeout = timeout;
    }


    @Override
    public void execute() {
        super.execute();
        swerve.driveRobotOriented(new ChassisSpeeds(vX, vY, omega));
    }

    @Override
    public boolean isFinished() {
        return super.isTimeoutExceeded(timeout);
    }
}
