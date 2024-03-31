package frc.robot.commands.auto;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class Score4SpeakerAutoCommand extends BaseAutoCommand {

    public Score4SpeakerAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        JackmanVisionSubsystem jackman, LightingSubsystem lighting, double delay, int noteCount) {
        super(swerve, armSubsystem, jackman, lighting);

        // start
        addCommands(log("Starting Auto"));
        addCommands(wait(delay));

        // loaded
        if (noteCount > 0) {
            addCommands(scoreSpeaker());
        }

        // wolverine
        if (noteCount > 1) {
            sequenceScoreWolverine();
        }

        // barnum
        if (noteCount > 2) {
            sequenceScoreBarnum();
        }

        // valjean
        if (noteCount > 3) {
            sequenceScoreValjean();
        }

        // Exit Zone
        switch (noteCount) {
        case 0:
        case 1:
        case 2:
            sequenceExitSourceSide();
            break;
        case 3:
            sequenceExitMiddle();
            break;
        case 4:
            sequenceExitAmpSide();
            break;
        default:
            break;
        }

        // end
        addCommands(log("Auto Complete"));
    }
}