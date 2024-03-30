package frc.robot.commands.auto;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

import static frc.robot.Constants.UsefulPoses.*;

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
            addCommands(
                wait(1.25).andThen(intake())
                    .alongWith(
                        driveTo(IN_FRONT_OF_WOLVERINE_BLUE, IN_FRONT_OF_WOLVERINE_RED).andThen(driveToNote(1))));
            // todo: maybe remove this reverse code
//            addCommands(driveRobotOriented(-1.0, 0, 0, 0.4));
            addCommands(scoreSpeaker());
        }

        // barnum
        if (noteCount > 2) {
            addCommands(
                intake()
                    .alongWith(
                        faceBarnum().andThen(driveToNote(1))));
            addCommands(scoreSpeaker());
        }

        // valjean
        if (noteCount > 3) {
            addCommands(
                intake()
                    .alongWith(
                        faceValjean().andThen(driveToNote(1))));
            addCommands(scoreSpeaker());
        }

        // Exit Zone
        switch (noteCount) {
        case 0:
        case 1:
        case 2:
            addCommands(driveTo(AFTER_WOLVERINE_AUTO_BLUE, AFTER_WOLVERINE_AUTO_RED));
            addCommands(driveTo(PARK_AFTER_WOLVERINE_AUTO_BLUE, PARK_AFTER_WOLVERINE_AUTO_RED));
            break;
        case 3:
            addCommands(driveTo(PARK_AFTER_BARNUM_AUTO_BLUE, PARK_AFTER_BARNUM_AUTO_RED));
            break;
        case 4:
            addCommands(driveTo(PARK_AFTER_VALJEAN_AUTO_BLUE, PARK_AFTER_VALJEAN_AUTO_RED));
            break;
        default:
            break;
        }

        // end
        addCommands(log("Auto Complete"));
    }
}