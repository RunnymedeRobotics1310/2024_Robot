package frc.robot.commands.auto;


import static frc.robot.Constants.BotTarget.BLUE_NOTE_WOLVERINE;
import static frc.robot.Constants.BotTarget.RED_NOTE_WOLVERINE;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.ShootSpeakerFromPodiumCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class Score2SpeakerVisualAutoCommand extends SequentialCommandGroup {

    public Score2SpeakerVisualAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay) {


        // In this case we need to adjust the pose and take off 70cm (on a diagonal from where the
        // bot will start by speaker) so it doesn't ram the podium
        Translation2d blueNote2Loc       = BLUE_NOTE_WOLVERINE.getLocation().toTranslation2d()
            .plus(new Translation2d(-0.48, 0.48));
        Translation2d redNote2Loc        = RED_NOTE_WOLVERINE.getLocation().toTranslation2d().plus(new Translation2d(0.48, 0.48));

        // Transition Away from Stage/Podium before moving to final
        Pose2d        blueTransitionPose = new Pose2d(BLUE_NOTE_WOLVERINE.getLocation().getX(), 1.5, new Rotation2d());
        Pose2d        redTransitionPose  = new Pose2d(RED_NOTE_WOLVERINE.getLocation().getX(), 1.5, new Rotation2d());

        // Final resting place
        Pose2d        blueFinishPose     = new Pose2d(4, 1.5, new Rotation2d());
        Pose2d        redFinishPose      = new Pose2d(12.54, 1.8, new Rotation2d());


        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        // TODO: replace FakeScoreSpeakerCommand

        /* ***AUTO PATTERN*** */

        /* Note 1 */
        // back up to not hit the speaker while rotating
        addCommands(new ShootCommand(armSubsystem, lighting));
        addCommands(new DriveToPositionCommand(swerve,
            BotTarget.BLUE_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(1.6, 0)),
            BotTarget.RED_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(-1.6, 0))));

        /* Note 2 */
        addCommands(new WaitCommand(1.3)
            .deadlineWith(new StartIntakeCommand(armSubsystem, lighting)));
        addCommands(new StartIntakeCommand(armSubsystem, lighting)
            .deadlineWith(new DriveToPositionFacingCommand(swerve, blueNote2Loc, redNote2Loc, MAX_TRANSLATION_SPEED_MPS)));
        // todo: the above speed was the default and is probably WAY too fast. Try 1.5.
        addCommands(new CompactCommand(armSubsystem));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve));
        addCommands(new ShootSpeakerFromPodiumCommand(armSubsystem, lighting));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, blueTransitionPose, redTransitionPose));
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
