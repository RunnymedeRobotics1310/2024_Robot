package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.swervedrive.DriveToNoteCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.RotateToPlacedNoteCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class Score4SpeakerAutoCommand extends SequentialCommandGroup {

    public Score4SpeakerAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, HughVisionSubsystem hugh,
        JackmanVisionSubsystem jackman) {

        final Pose2d blueFinishPose = new Pose2d(new Translation2d(3.5, 7), new Rotation2d());
        final Pose2d redFinishPose  = new Pose2d(new Translation2d(13.04, 7), new Rotation2d());


        addCommands(new LogMessageCommand("Starting Auto"));

        /* ***AUTO PATTERN*** */


        // TODO: replace FakeScoreSpeakerCommand

        /* Note 1 */
        // back up to not hit the speaker while rotating
        addCommands(new DriveToPositionCommand(swerve,
            BotTarget.BLUE_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(1.6, 0)),
            BotTarget.RED_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(-1.6, 0))));
        addCommands(new ShootCommand(armSubsystem));


        /* Note 2 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_WOLVERINE, BotTarget.RED_NOTE_WOLVERINE));
        addCommands(new StartIntakeCommand(armSubsystem)
            .alongWith(new DriveToNoteCommand(swerve, armSubsystem, jackman, 0.25)));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ShootCommand(armSubsystem));


        /* Note 3 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_BARNUM, BotTarget.RED_NOTE_BARNUM));
        addCommands(new StartIntakeCommand(armSubsystem)
            .alongWith(new DriveToNoteCommand(swerve, armSubsystem, jackman, 0.25)));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ShootCommand(armSubsystem));

        /* Note 4 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_VALJEAN, BotTarget.RED_NOTE_VALJEAN));
        addCommands(new StartIntakeCommand(armSubsystem)
            .alongWith(new DriveToNoteCommand(swerve, armSubsystem, jackman, 0.25)));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ShootCommand(armSubsystem));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
