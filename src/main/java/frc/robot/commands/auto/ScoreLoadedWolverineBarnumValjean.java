package frc.robot.commands.auto;

import static frc.robot.Constants.FieldConstants.BLUE_BARNUM;
import static frc.robot.Constants.FieldConstants.BLUE_BARNUM_SHOT;
import static frc.robot.Constants.FieldConstants.BLUE_VALJEAN;
import static frc.robot.Constants.FieldConstants.RED_BARNUM;
import static frc.robot.Constants.FieldConstants.RED_BARNUM_SHOT;
import static frc.robot.Constants.FieldConstants.RED_VALJEAN;
import static frc.robot.Constants.UsefulPoses.IN_FRONT_OF_WOLVERINE_BLUE;
import static frc.robot.Constants.UsefulPoses.IN_FRONT_OF_WOLVERINE_RED;
import static frc.robot.Constants.UsefulPoses.WOLVERINE_PICKUP_BLUE;
import static frc.robot.Constants.UsefulPoses.WOLVERINE_PICKUP_RED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.CompactFromIntakeCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.ShootSpeakerFromAnywhereCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.DriveToPositionFacingCommand;
import frc.robot.commands.swervedrive.RotateToLocationCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.commands.swervedrive.SimpleDriveRobotOrientedCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class ScoreLoadedWolverineBarnumValjean extends SequentialCommandGroup {

    public ScoreLoadedWolverineBarnumValjean(SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay, int noteCount) {

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
                new StartIntakeCommand(armSubsystem, lighting, null)
                    .deadlineWith(new DriveToPositionCommand(swerve, WOLVERINE_PICKUP_BLUE, WOLVERINE_PICKUP_RED, 1.5)));
            // todo: maybe remove this reverse code
            addCommands(new SimpleDriveRobotOrientedCommand(swerve, -1.0, 0, 0, 0.4));
            addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve));
            // todo:replace with shoot from anywhere
            addCommands(new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting));
        }

        // barnum
        if (noteCount > 2) {
            Command arm   = new CompactCommand(armSubsystem)
                .andThen(new StartIntakeCommand(armSubsystem, lighting, null));
            Command drive = new RotateToLocationCommand(swerve, BLUE_BARNUM, RED_BARNUM)
                .andThen(new DriveToPositionFacingCommand(swerve, BLUE_BARNUM, RED_BARNUM, 1.5));
            addCommands(arm.deadlineWith(drive));

            addCommands(new DriveToPositionCommand(swerve, BLUE_BARNUM_SHOT, RED_BARNUM_SHOT));

            addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve)
                .alongWith(new CompactFromIntakeCommand(armSubsystem, false)));
            // todo:replace with shoot from anywhere
            addCommands(new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting));
            addCommands(new CompactCommand(armSubsystem));

        }

        // valjean
        if (noteCount > 3) {
            Command arm   = new CompactCommand(armSubsystem)
                .andThen(new StartIntakeCommand(armSubsystem, lighting, null));
            Command drive = new RotateToLocationCommand(swerve, BLUE_VALJEAN, RED_VALJEAN)
                .andThen(new DriveToPositionFacingCommand(swerve, BLUE_VALJEAN, RED_VALJEAN, 1.5));
            addCommands(arm.deadlineWith(drive));


            addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve)
                .alongWith(new CompactFromIntakeCommand(armSubsystem, false)));
            // todo:replace with shoot from anywhere
            addCommands(new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting));
            addCommands(new CompactCommand(armSubsystem));
        }

        // end
        addCommands(new LogMessageCommand("Auto Complete"));

    }
}
