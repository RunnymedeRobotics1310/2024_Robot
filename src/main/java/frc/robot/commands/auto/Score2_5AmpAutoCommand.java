package frc.robot.commands.auto;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Score2_5AmpAutoCommand extends BaseAutoCommand {

    public Score2_5AmpAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, JackmanVisionSubsystem jackman,
        LightingSubsystem lighting, double delay) {
        super(swerve, armSubsystem, jackman, lighting);

        addCommands(log("Starting Auto"));
        addCommands(wait(delay));

        /* Note 1 */
        addCommands(scoreAmp());

        /* Note 2 */
        addCommands(goGetValjean());
        addCommands(scoreAmp());

        /* Note 3 */
        addCommands(goGetBarnum());
        addCommands(driveToAmp().andThen(aimAmp()));
        addCommands(log("Auto Complete"));

    }
}