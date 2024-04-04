package frc.robot.commands.auto;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class TheSpeakerAuto extends BaseAutoCommand {

    public TheSpeakerAuto(SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay, int noteCount) {
        super(swerve, armSubsystem, jackman, lighting);

        enum Note {
            Loaded, Wolverine, Barnum, Valjean
        }

        Note lastNote = null;

        // start
        addCommands(log("Starting Auto"));
        addCommands(wait(delay));

        // loaded
        if (noteCount > 0) {
            addCommands(scoreSpeaker(true));
            addCommands(compactCommand());
            lastNote = Note.Loaded;
        }

        // wolverine
        if (noteCount > 1) {
            addCommands(goGetWolverine());
            addCommands(scoreSpeaker());
            lastNote = Note.Wolverine;
        }

        // barnum
        if (noteCount > 2) {
            addCommands(goGetBarnum());
            addCommands(scoreSpeaker());
            lastNote = Note.Barnum;
        }

        // valjean
        if (noteCount > 3) {
            addCommands(goGetValjean());
            addCommands(scoreSpeaker());
            lastNote = Note.Valjean;
        }

        // Exit Zone
        switch (lastNote) {
        case Valjean:
            addCommands(goGetNote5());
            break;
        case Barnum:
            addCommands(goGetCenterNote());
            break;
        case Wolverine:
        case Loaded:
        default:
            addCommands(exitSourceSide());
            break;
        }

        // end
        addCommands(log("Auto Complete"));
    }
}