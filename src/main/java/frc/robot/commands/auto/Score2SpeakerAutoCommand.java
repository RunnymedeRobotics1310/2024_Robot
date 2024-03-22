package frc.robot.commands.auto;


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
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

import static frc.robot.Constants.BotTarget.*;

public class Score2SpeakerAutoCommand extends SequentialCommandGroup {

    public Score2SpeakerAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, HughVisionSubsystem hugh,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay) {

        Translation2d blueNote2Loc       = BLUE_NOTE_WOLVERINE.getLocation().toTranslation2d()
            .plus(new Translation2d(-0.48, 0.48));
        Translation2d redNote2Loc        = RED_NOTE_WOLVERINE.getLocation().toTranslation2d().plus(new Translation2d(0.48, 0.48));


        Pose2d        blueFinishPose     = new Pose2d(4, 1.5, new Rotation2d());
        Pose2d        redFinishPose      = new Pose2d(12.54, 1.8, new Rotation2d());

        Pose2d        blueTransitionPose = new Pose2d(BLUE_NOTE_WOLVERINE.getLocation().getX(), 1.5, new Rotation2d());
        Pose2d        redTransitionPose  = new Pose2d(RED_NOTE_WOLVERINE.getLocation().getX(), 1.5, new Rotation2d());



        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));


        /* Note 1 */
        // back up to not hit the speaker while rotating
        addCommands(new ShootCommand(armSubsystem, lighting));
        addCommands(new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, .4));
//        addCommands(new DriveToPositionCommand(swerve,
//            BotTarget.BLUE_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(1.6, 0)),
//            BotTarget.RED_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(-1.6, 0))));


        /* Note 2 */
        addCommands(new RotateToLocationCommand(swerve,
            BotTarget.BLUE_NOTE_WOLVERINE.getLocation().toTranslation2d().plus(new Translation2d(0.3, 0.0)),
            BotTarget.RED_NOTE_WOLVERINE.getLocation().toTranslation2d().plus(new Translation2d(-0.3, 0.0))));
//        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_WOLVERINE, BotTarget.RED_NOTE_WOLVERINE));
        addCommands(new WaitCommand(1.3)
            .deadlineWith(new StartIntakeCommand(armSubsystem, lighting)));
        addCommands(new StartIntakeCommand(armSubsystem, lighting)
//                .deadlineWith(new DriveToPositionFacingCommand(swerve, blueNote2Loc, redNote2Loc, blueNote2Loc, redNote2Loc)));
            .deadlineWith(new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 2)));

        addCommands(new CompactCommand(armSubsystem));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ShootSpeakerFromPodiumCommand(armSubsystem, lighting));

        /* Exit Zone */
//
//        addCommands(new DriveToPositionCommand(swerve, blueTransitionPose, redTransitionPose));
//        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
