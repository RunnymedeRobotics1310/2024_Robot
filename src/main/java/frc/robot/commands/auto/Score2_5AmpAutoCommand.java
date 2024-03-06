package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.auto.stubs.FakeScoreAmpCommand;
import frc.robot.commands.auto.stubs.FakeVisionNotePickupCommand;
import frc.robot.commands.swervedrive.DriveToNoteCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.RotateToPlacedNoteCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

import static frc.robot.Constants.UsefulPoses.SCORE_BLUE_AMP;
import static frc.robot.Constants.UsefulPoses.SCORE_RED_AMP;

public class Score2_5AmpAutoCommand extends SequentialCommandGroup {

    public Score2_5AmpAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, HughVisionSubsystem hugh, JackmanVisionSubsystem jackman) {

        addCommands(new LogMessageCommand("Starting Auto"));

        // TODO: replace FaKeScoreAmpCommand

        /* Note 1 */
        addCommands(new DriveToPositionCommand(swerve, SCORE_BLUE_AMP, SCORE_RED_AMP));
        addCommands(new FakeScoreAmpCommand());

        /* Note 2 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_VALJEAN, BotTarget.RED_NOTE_VALJEAN));
        addCommands(new StartIntakeCommand(armSubsystem)
                .deadlineWith(new DriveToNoteCommand(swerve, armSubsystem, jackman, .5)));
        addCommands(new DriveToPositionCommand(swerve, SCORE_BLUE_AMP, SCORE_RED_AMP));
        addCommands(new FakeScoreAmpCommand());

        /* Note 3 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_BARNUM, BotTarget.RED_NOTE_BARNUM));
        addCommands(new StartIntakeCommand(armSubsystem)
                .deadlineWith(new DriveToNoteCommand(swerve, armSubsystem, jackman, .5)));

        /* Exit zone & finish at amp */
        // addCommands(new DriveToPositionCommand(swerve,
        // new Pose2d(BotTarget.BLUE_NOTE_VALJEAN.getLocation().toTranslation2d(),
        // Rotation2d.fromDegrees(90)),
        // new Pose2d(BotTarget.RED_NOTE_VALJEAN.getLocation().toTranslation2d(),
        // Rotation2d.fromDegrees(90))));
        addCommands(new DriveToPositionCommand(swerve,
            new Pose2d(BotTarget.BLUE_NOTE_BARNUM.getLocation().toTranslation2d(), Rotation2d.fromDegrees(90)),
            new Pose2d(BotTarget.RED_NOTE_BARNUM.getLocation().toTranslation2d(), Rotation2d.fromDegrees(90))));
        addCommands(new DriveToPositionCommand(swerve, SCORE_BLUE_AMP, SCORE_RED_AMP));
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}