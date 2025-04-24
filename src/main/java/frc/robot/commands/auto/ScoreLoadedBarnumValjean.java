package frc.robot.commands.auto;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.*;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.DriveToPositionFacingCommand;
import frc.robot.commands.swervedrive.RotateToLocationCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class ScoreLoadedBarnumValjean extends BaseAutoCommand {

    public ScoreLoadedBarnumValjean(SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay) {
        super(swerve, armSubsystem, jackman, lighting);

        // start
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        // loaded
        addCommands(new ShootCommand(armSubsystem, lighting));

        // barnum
        addCommands(new WaitCommand(.8)
            .raceWith(new StartIntakeCommand(armSubsystem, lighting)));
        addCommands(new StartIntakeCommand(armSubsystem, lighting)
            .raceWith(new DriveToPositionCommand(swerve, BLUE_BARNUM, RED_BARNUM)));
        addCommands(new DriveToPositionCommand(swerve, BLUE_BARNUM_SHOT, RED_BARNUM_SHOT));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve)
            .alongWith(new CompactFromIntakeCommand(armSubsystem, false)
                .alongWith(new ReverseNoteCommand(armSubsystem))));
        addCommands(new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting));

        // valjean
        Command arm   = new CompactCommand(armSubsystem)
            .andThen(new StartIntakeCommand(armSubsystem, lighting));
        Command drive = new RotateToLocationCommand(swerve, BLUE_VALJEAN, RED_VALJEAN)
            .andThen(new DriveToPositionFacingCommand(swerve, BLUE_VALJEAN, RED_VALJEAN, 1.5));
        addCommands(arm.raceWith(drive));


        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve)
            .alongWith(new CompactFromIntakeCommand(armSubsystem, false)
                .alongWith(new ReverseNoteCommand(armSubsystem))));
        addCommands(new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting));
        addCommands(new CompactCommand(armSubsystem));

        // Exit
        addCommands(goGetNote5());

        // end
        addCommands(new LogMessageCommand("Auto Complete"));

    }
}
