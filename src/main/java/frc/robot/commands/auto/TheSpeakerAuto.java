package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.Constants.AutoConstants.Note;

public class TheSpeakerAuto extends BaseAutoCommand {

    public TheSpeakerAuto(SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, Note note1, Note note2, Note note3, double delay, int noteCount) {

        super(swerve, armSubsystem, jackman, lighting);


        Note lastNote = null;

        Command wolverine = goGetWolverine().andThen(scoreSpeaker());
        Command barnum = goGetBarnum().andThen(scoreSpeaker());
        Command valjean = goGetValjean().andThen(scoreSpeaker());

        // start
        addCommands(log("Starting Auto"));
        addCommands(wait(delay));


        // loaded (Note 0)
            addCommands(scoreSpeaker());
            lastNote = Note.Loaded;


        // Note 1
        switch (note1) {
            case Valjean:
                addCommands(valjean);
                break;
            case Barnum:
                addCommands(barnum);
                break;
            case Wolverine:
                addCommands(wolverine);
            case None:
            default:
                break;
        }

        // Note 2
        switch (note2) {
            case Valjean:
                addCommands(valjean);
                break;
            case Barnum:
                addCommands(barnum);
                break;
            case Wolverine:
                addCommands(wolverine);
            case None:
                lastNote = note1;
            default:
                break;
        }

        // Note 3
        switch (note3) {
            case Valjean:
                addCommands(valjean);
                break;
            case Barnum:
                addCommands(barnum);
                break;
            case Wolverine:
                addCommands(wolverine);
            case None:
                lastNote = note2;
            default:
                break;
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