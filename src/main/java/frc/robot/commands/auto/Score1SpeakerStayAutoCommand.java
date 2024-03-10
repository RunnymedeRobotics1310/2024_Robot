package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ManualShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score1SpeakerStayAutoCommand extends SequentialCommandGroup {

    public Score1SpeakerStayAutoCommand(SwerveSubsystem swerve, ArmSubsystem arm, HughVisionSubsystem hugh) {


        addCommands(new LogMessageCommand("Starting Auto"));

        /* ***AUTO PATTERN*** */

        /* Note 1 */

        // IMPORTANT: line up with speaker

        // addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ManualShootCommand(arm));


        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}