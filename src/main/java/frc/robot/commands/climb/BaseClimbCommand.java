package frc.robot.commands.climb;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

abstract class BaseClimbCommand extends LoggingCommand {

    protected final ClimbSubsystem  climbSubsystem;
    protected final SwerveSubsystem driveSubsystem;

    protected BaseClimbCommand(ClimbSubsystem climb, SwerveSubsystem drive) {
        this.climbSubsystem = climb;
        this.driveSubsystem = drive;
        addRequirements(climb);
    }

}
