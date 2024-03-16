package frc.robot.commands.auto;


import static frc.robot.Constants.UsefulPoses.SCORE_BLUE_AMP;
import static frc.robot.Constants.UsefulPoses.SCORE_RED_AMP;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.auto.stubs.FakeScoreAmpCommand;
import frc.robot.commands.auto.stubs.FakeVisionNotePickupCommand;
import frc.robot.commands.swervedrive.DriveToNoteCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.RotateToPlacedNoteCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class Score2AmpAutoCommand extends SequentialCommandGroup {

    public Score2AmpAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, HughVisionSubsystem hugh,
        JackmanVisionSubsystem jackman, double delay) {

        Pose2d blueFinishPose = new Pose2d(4, 7.0, new Rotation2d(90));
        Pose2d redFinishPose  = new Pose2d(12.54, 7.0, new Rotation2d());


        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));


        // TODO: replace FakeScoreAmpCommand

        /* Note 1 */
        addCommands(new DriveToPositionCommand(swerve, SCORE_BLUE_AMP, SCORE_RED_AMP));
        addCommands(new FakeScoreAmpCommand());

        /* Note 2 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_VALJEAN, BotTarget.RED_NOTE_VALJEAN));
        addCommands(new StartIntakeCommand(armSubsystem)
            .deadlineWith(new DriveToNoteCommand(swerve, armSubsystem, jackman, .5)));
        addCommands(new DriveToPositionCommand(swerve, SCORE_BLUE_AMP, SCORE_RED_AMP));
        addCommands(new FakeScoreAmpCommand());

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}