package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Score1SpeakerAutoCommand extends SequentialCommandGroup {

    public Score1SpeakerAutoCommand(SwerveSubsystem swerve, ArmSubsystem arm, LightingSubsystem lighting, double delay) {

        Pose2d blueFinishPose  = new Pose2d(4, 1.5, new Rotation2d());
        Pose2d redFinishPose   = new Pose2d(12.54, 1.8, new Rotation2d());

        Pose2d blueTransitPose = new Pose2d(2, 3, new Rotation2d());
        Pose2d redTransitPose  = new Pose2d(10.54, 3, new Rotation2d());


        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        /* Note 1 */

        // IMPORTANT: line up with speaker
        addCommands(new ShootCommand(arm, lighting));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, blueTransitPose, redTransitPose));
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}