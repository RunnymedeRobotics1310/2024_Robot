package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ManualShootCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

    public ExitZoneAutoCommand(SwerveSubsystem swerve, ArmSubsystem arm, HughVisionSubsystem hugh) {

        Pose2d blueFinishPose = new Pose2d(4, 1.5, new Rotation2d());
        Pose2d redFinishPose  = new Pose2d(12.54, 1.8, new Rotation2d());


        addCommands(new LogMessageCommand("Starting Auto"));


        // TODO: replace FakeScoreSpeakerCommand

        /* ***AUTO PATTERN*** */

        /* Note 1 */


        /* Exit Zone */
        addCommands(new DriveRobotOrientedCommand(swerve, new Translation2d(3, 0), new Rotation2d()));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}