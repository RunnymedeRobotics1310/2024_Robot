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

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.FieldConstants.BLUE_VALJEAN;

public class ScoreLoadedBarnumValjean extends SequentialCommandGroup {

    public ScoreLoadedBarnumValjean(SwerveSubsystem swerve, ArmSubsystem armSubsystem, HughVisionSubsystem hugh,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay) {

        // start
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        // loaded
        addCommands(new ShootCommand(armSubsystem, lighting));

        // barnum
        addCommands(new StartIntakeCommand(armSubsystem, lighting)
            .deadlineWith(new DriveToPositionFacingCommand(swerve, BLUE_BARNUM, RED_BARNUM)));
        addCommands(new ShootSpeakerFromPodiumCommand(armSubsystem, lighting));

        // valjean
        addCommands(new CompactCommand(armSubsystem)
            .alongWith(new RotateToLocationCommand(swerve, BLUE_VALJEAN, BLUE_VALJEAN)));
        addCommands(new StartIntakeCommand(armSubsystem, lighting)
            .deadlineWith(new DriveToPositionFacingCommand(swerve, BLUE_VALJEAN, BLUE_VALJEAN)));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new ShootSpeakerFromPodiumCommand(armSubsystem, lighting));


        // end
        addCommands(new LogMessageCommand("Auto Complete"));

    }
}
