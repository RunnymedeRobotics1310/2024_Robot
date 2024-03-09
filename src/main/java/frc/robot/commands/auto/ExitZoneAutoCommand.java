package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

    public ExitZoneAutoCommand(SwerveSubsystem swerve) {


        addCommands(new LogMessageCommand("Starting Auto"));

        /* ***AUTO PATTERN*** */

        /* Exit Zone */
        addCommands(new DriveRobotOrientedCommand(swerve, new Translation2d(3, 0), new Rotation2d()));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}