package frc.robot.commands.auto;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class ClearCentreAutoCommand extends BaseAutoCommand {

    public ClearCentreAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay) {
        super(swerve, armSubsystem, jackman, lighting);

        // start
        addCommands(log("Starting Auto"));
        addCommands(wait(delay));

        // loaded
        addCommands(scoreSpeaker(true));
        addCommands(compactCommand());

        // Centre 1
        addCommands(goGetCentreNoteX(1));
        addCommands(scoreSpeaker());

        // Centre 2
        addCommands(goGetCentreNoteX(2));
        addCommands(scoreSpeaker());

        // Centre 2
        addCommands(goGetCentreNoteX(3));
        addCommands(scoreSpeaker());

        // Centre 2
        addCommands(goGetCentreNoteX(4));
        addCommands(scoreSpeaker());
        addCommands(compactCommand());

        // end
        addCommands(log("Auto Complete"));

    }
}
