package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.swervedrive.DriveToNoteCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.RotateToPlacedNoteCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

import static frc.robot.Constants.BotTarget.*;

public class Score2SpeakerAutoCommand extends SequentialCommandGroup {

    public Score2SpeakerAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, HughVisionSubsystem hugh,
                                    JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay) {


        Pose2d blueFinishPose     = new Pose2d(4, 1.5, new Rotation2d());
        Pose2d redFinishPose      = new Pose2d(12.54, 1.8, new Rotation2d());

        Pose2d blueTransitionPose = new Pose2d(BLUE_NOTE_WOLVERINE.getLocation().getX(), 1.5, new Rotation2d());
        Pose2d redTransitionPose  = new Pose2d(RED_NOTE_WOLVERINE.getLocation().getX(), 1.5, new Rotation2d());


        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        // TODO: replace FakeScoreSpeakerCommand

        /* ***AUTO PATTERN*** */

        /* Note 1 */
        // back up to not hit the speaker while rotating
        addCommands(new DriveToPositionCommand(swerve,
            BotTarget.BLUE_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(1.6, 0)),
            BotTarget.RED_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(-1.6, 0))));
        addCommands(new ShootCommand(armSubsystem, lighting));

        /* Note 3 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_WOLVERINE, BotTarget.RED_NOTE_WOLVERINE));
        addCommands(new StartIntakeCommand(armSubsystem, lighting)
            .alongWith(new DriveToNoteCommand(swerve, armSubsystem, jackman, 0.25)));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ShootCommand(armSubsystem, lighting));

        /* Exit Zone */

        addCommands(new DriveToPositionCommand(swerve, blueTransitionPose, redTransitionPose));
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
