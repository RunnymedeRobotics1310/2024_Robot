package frc.robot.commands.auto;


import static frc.robot.Constants.FieldConstants.CENTRE_NOTE_2;
import static frc.robot.Constants.FieldConstants.FIELD_EXTENT_METRES_X;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.CompactFromIntakeCommand;
import frc.robot.commands.arm.ReverseNoteCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.swervedrive.DriveToNoteCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class Score1SpeakerAutoCommand extends SequentialCommandGroup {

    public Score1SpeakerAutoCommand(SwerveSubsystem swerve, ArmSubsystem arm, JackmanVisionSubsystem jackman,
        LightingSubsystem lighting, double delay) {

        Pose2d blueFinishPose  = new Pose2d(4, 1, new Rotation2d());
        Pose2d redFinishPose   = new Pose2d(FIELD_EXTENT_METRES_X - 4, 1, new Rotation2d());

        Pose2d blueTransitPose = new Pose2d(2, 3, new Rotation2d());
        Pose2d redTransitPose  = new Pose2d(FIELD_EXTENT_METRES_X - 2, 3, new Rotation2d());

        Pose2d bluePickupPose  = new Pose2d(CENTRE_NOTE_2.getX() - 1.5, CENTRE_NOTE_2.getY(), new Rotation2d());
        Pose2d redPickupPose   = new Pose2d(CENTRE_NOTE_2.getX() + 1.5, CENTRE_NOTE_2.getY(), new Rotation2d());

        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        /* Note 1 */

        // IMPORTANT: line up with speaker
        addCommands(new ShootCommand(arm, lighting));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, blueTransitPose, redTransitPose));
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        /* Pick up Note */
        addCommands(new DriveToPositionCommand(swerve, bluePickupPose, redPickupPose));
        addCommands(new DriveToNoteCommand(swerve, lighting, arm, jackman, 1)
            .alongWith(new StartIntakeCommand(arm, lighting, null)));

        /* Drive Back */
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose)
            .alongWith(new CompactFromIntakeCommand(arm, false)
                .alongWith(new ReverseNoteCommand(arm))));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}