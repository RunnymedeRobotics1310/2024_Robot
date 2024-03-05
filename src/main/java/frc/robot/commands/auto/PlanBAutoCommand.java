package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

import static frc.robot.Constants.UsefulPoses.SCORE_BLUE_AMP;
import static frc.robot.Constants.UsefulPoses.SCORE_RED_AMP;

public class PlanBAutoCommand extends SequentialCommandGroup {

    public PlanBAutoCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {

        Pose2d blueFinishPose = new Pose2d(4, 7.7, new Rotation2d(90));
        Pose2d redFinishPose  = new Pose2d(12.54, 7.4, new Rotation2d());
        Pose2d blueStartPose = new Pose2d(new Translation2d(46.355, 777.185), Rotation2d.fromDegrees(270));
        Pose2d redStartPose = new Pose2d(new Translation2d(1607.645, 777.185), Rotation2d.fromDegrees(270));

        //Start this auto in corner.
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new ResetOdometryCommand(swerve, blueStartPose, redStartPose));

        addCommands(new DriveToPositionCommand(swerve, SCORE_BLUE_AMP, SCORE_RED_AMP, 1));
        addCommands(new DriveRobotOrientedCommand(swerve, new Translation2d(-.1, 0), new Rotation2d()));
        addCommands(new DriveRobotOrientedCommand(swerve, new Translation2d(.1, 0), new Rotation2d()));
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        addCommands(new LogMessageCommand("Auto Complete"));

        }
}