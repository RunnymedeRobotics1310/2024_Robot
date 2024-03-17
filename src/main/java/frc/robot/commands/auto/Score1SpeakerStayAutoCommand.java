package frc.robot.commands.auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score1SpeakerStayAutoCommand extends SequentialCommandGroup {

    public Score1SpeakerStayAutoCommand(SwerveSubsystem swerve, ArmSubsystem arm, HughVisionSubsystem hugh, LightingSubsystem lighting, double delay) {


        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));
        /* ***AUTO PATTERN*** */

        /* Note 1 */

        // IMPORTANT: line up with speaker

        // addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ShootCommand(arm, lighting));


        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}