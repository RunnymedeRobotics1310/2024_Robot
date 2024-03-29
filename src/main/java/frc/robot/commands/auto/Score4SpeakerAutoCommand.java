package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.*;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.UsefulPoses.*;

public class Score4SpeakerAutoCommand extends SequentialCommandGroup {

    public Score4SpeakerAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay, int noteCount) {

        final Pose2d blueFinishPose = new Pose2d(new Translation2d(3.5, 7), new Rotation2d());
        final Pose2d redFinishPose  = new Pose2d(new Translation2d(13.04, 7), new Rotation2d());


        // start
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        // loaded
        if (noteCount > 0) {
            // Shoot from right at the speaker (todo: replace with from anywhere)
            addCommands(new ShootCommand(armSubsystem, lighting));
        }

        // wolverine
        if (noteCount > 1) {
            addCommands(new DriveToPositionCommand(swerve, IN_FRONT_OF_WOLVERINE_BLUE, IN_FRONT_OF_WOLVERINE_RED));
            addCommands(
                new StartIntakeCommand(armSubsystem, lighting)
                    .deadlineWith(new DriveToNoteCommand(swerve, lighting, armSubsystem, jackman, 0.25)));
            // todo: maybe remove this reverse code
            addCommands(new SimpleDriveRobotOrientedCommand(swerve, -1.0, 0, 0, 0.4));
            addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve));
            // todo:replace with shoot from anywhere
            addCommands(new ShootSpeakerFromPodiumCommand(armSubsystem, lighting));
        }

        // barnum
        if (noteCount > 2) {
            Command arm   = new CompactCommand(armSubsystem)
                .andThen(new StartIntakeCommand(armSubsystem, lighting));
            Command drive = new RotateToLocationCommand(swerve, BLUE_BARNUM, RED_BARNUM)
                .andThen(new DriveToNoteCommand(swerve, lighting, armSubsystem, jackman, 0.25));
            addCommands(arm.deadlineWith(drive));

            addCommands(new DriveToPositionCommand(swerve, BLUE_BARNUM_SHOT, RED_BARNUM_SHOT));

            addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve)
                .alongWith(new CompactFromIntakeCommand(armSubsystem, false)));
            // todo:replace with shoot from anywhere
            addCommands(new ShootSpeakerFromPodiumCommand(armSubsystem, lighting));
            addCommands(new CompactCommand(armSubsystem));

        }

        // valjean
        if (noteCount > 3) {
            Command arm   = new CompactCommand(armSubsystem)
                .andThen(new StartIntakeCommand(armSubsystem, lighting));
            Command drive = new RotateToLocationCommand(swerve, BLUE_VALJEAN, RED_VALJEAN)
                .andThen(new DriveToNoteCommand(swerve, lighting, armSubsystem, jackman, 0.25));
            addCommands(arm.deadlineWith(drive));


            addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve)
                .alongWith(new CompactFromIntakeCommand(armSubsystem, false)));
            // todo:replace with shoot from anywhere
            addCommands(new ShootSpeakerFromPodiumCommand(armSubsystem, lighting));
            addCommands(new CompactCommand(armSubsystem));
        }

        // Exit Zone
        if (noteCount > 3) {
            addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));
        }

        // end
        addCommands(new LogMessageCommand("Auto Complete"));

    }
}
