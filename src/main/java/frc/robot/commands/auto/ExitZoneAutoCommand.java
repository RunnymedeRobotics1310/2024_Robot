package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

    public ExitZoneAutoCommand(SwerveSubsystem swerve, double delay) {


        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));
        /* ***AUTO PATTERN*** */

        /* Exit Zone */
        addCommands(new ResetOdometryCommand(swerve, new Pose2d(new Translation2d(), new Rotation2d())));
        addCommands(new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 3));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}