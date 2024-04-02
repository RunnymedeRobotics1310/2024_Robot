package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.FieldConstants.IN_FRONT_OF_BLUE_VALJEAN;
import static frc.robot.Constants.FieldConstants.IN_FRONT_OF_RED_VALJEAN;

public class Score2_5AmpAutoCommand extends BaseAutoCommand {

    public Score2_5AmpAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, JackmanVisionSubsystem jackman,
        LightingSubsystem lighting, double delay) {
        super(swerve, armSubsystem, jackman, lighting);

        addCommands(log("Starting Auto"));
        addCommands(wait(delay));

        /* Note 1 */
        addCommands(scoreAmp());

        /* Note 2 */
        Pose2d ifvBlue = new Pose2d(IN_FRONT_OF_BLUE_VALJEAN, new Rotation2d(0.0));
        Pose2d ifvRed  = new Pose2d(IN_FRONT_OF_RED_VALJEAN, new Rotation2d(180));
        addCommands(driveTo(ifvBlue, ifvRed));
        addCommands(goGetValjean());
        addCommands(scoreAmp());

        /* Note 3 */
        addCommands(goGetBarnum());
        addCommands(driveToAmp().andThen(aimAmp()));
        addCommands(log("Auto Complete"));

    }
}