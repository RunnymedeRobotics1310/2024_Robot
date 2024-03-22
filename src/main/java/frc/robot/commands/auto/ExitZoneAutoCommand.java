package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

    public ExitZoneAutoCommand(SwerveSubsystem swerve, double delay) {

        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        addCommands(new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 3));

        addCommands(new LogMessageCommand("Auto Complete"));
    }
}