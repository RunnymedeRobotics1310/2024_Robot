package frc.robot.commands.auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import static frc.robot.Constants.UsefulPoses.IN_FRONT_OF_WOLVERINE_BLUE;
import static frc.robot.Constants.UsefulPoses.IN_FRONT_OF_WOLVERINE_RED;

public class ScoreLoadedWolverine extends SequentialCommandGroup {

    public ScoreLoadedWolverine(SwerveSubsystem swerve, ArmSubsystem armSubsystem, HughVisionSubsystem hugh,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay) {

        // start
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        // loaded
        addCommands(new ShootCommand(armSubsystem, lighting));

        // wolverine
        addCommands(new DriveToPositionCommand(swerve, IN_FRONT_OF_WOLVERINE_BLUE, IN_FRONT_OF_WOLVERINE_RED));
        addCommands(
            new StartIntakeCommand(armSubsystem, lighting)
                .deadlineWith(new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 1.5)));
        addCommands(new CompactCommand(armSubsystem));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ShootSpeakerFromPodiumCommand(armSubsystem, lighting));

        // end
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
