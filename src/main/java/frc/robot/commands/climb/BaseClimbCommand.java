package frc.robot.commands.climb;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class BaseClimbCommand extends LoggingCommand {

    private final ClimbSubsystem  climbSubsystem;
    private final SwerveSubsystem driveSubsystem;

    public BaseClimbCommand(ClimbSubsystem climb, SwerveSubsystem drive) {
        this.climbSubsystem = climb;
        this.driveSubsystem = drive;
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Pull the robot up (arms down, -ve motor speed) using operator triggers
        // NOTE: Left trigger is already negative.
        // Bumpers lift the arms up in order to catch the chain
    }
}
