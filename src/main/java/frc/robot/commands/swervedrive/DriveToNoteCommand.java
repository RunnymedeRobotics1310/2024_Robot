package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToNoteCommand extends BaseDriveCommand {
    private final JackmanVisionSubsystem jackman;
    private double                       speed;

    // todo: fixme: specify unit in speed param name (e.g. speedRPM, speedDegPerSec, etc.)
    public DriveToNoteCommand(SwerveSubsystem drive, ArmSubsystem arm, JackmanVisionSubsystem jackman, double speed) {

        super(drive);
        this.jackman = jackman;
        this.speed   = speed;
        addRequirements(jackman);
    }

    public void execute() {
        super.execute();

        Rotation2d robotRelativeOffset = jackman.getNoteOffset();

        if (robotRelativeOffset != null) {

            if (Math.abs(robotRelativeOffset.getDegrees()) > 10) {
                speed = 0;
            }

            Rotation2d omega = computeOmega(robotRelativeOffset);
            swerve.driveRobotOriented(new ChassisSpeeds(speed, 0, omega.getRadians()));
        }

    }

    @Override
    public boolean isFinished() {
        // IMPORTANT: This command will not end itself, it is designed to be cancelled by a
        // collaborating command. Specifically, it is primarily used in "deadline with intake
        // command" which will cancel this command when the intake is ready to intake the note.
        return false;
    }

}
