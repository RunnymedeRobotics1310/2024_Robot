package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.IntakeWithVision;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.LightingConstants.SIGNAL;


public class DriveToNoteCommand extends BaseDriveCommand {
    private final JackmanVisionSubsystem jackman;

    private final LightingSubsystem      lighting;
    private final ArmSubsystem           arm;
    private final double                 speedMPS;


    public DriveToNoteCommand(SwerveSubsystem drive, LightingSubsystem lighting, ArmSubsystem arm, JackmanVisionSubsystem jackman,
        double speedMPS) {

        super(drive);
        this.lighting = lighting;
        this.jackman  = jackman;
        this.speedMPS = speedMPS;
        this.arm      = arm;
        addRequirements(jackman);
    }

    @Override
    public void initialize() {
        super.initialize();
        lighting.addPattern(SIGNAL, IntakeWithVision.getInstance());
    }


    @Override
    public void execute() {
        super.execute();

        Rotation2d robotRelativeOffset = jackman.getNoteOffset();

        if (robotRelativeOffset != null) {
            double setSpeed = speedMPS;

            if (Math.abs(robotRelativeOffset.getDegrees()) > 7) {
                Rotation2d omega = computeOmegaForOffset(robotRelativeOffset);
                swerve.driveRobotOriented(new ChassisSpeeds(0, 0, -omega.getRadians()));

            }

            else {
                swerve.driveRobotOriented(new ChassisSpeeds(setSpeed, 0, 0));
            }
        }

    }

    @Override
    public boolean isFinished() {
        Rotation2d robotRelativeOffset = jackman.getNoteOffset();
        if (robotRelativeOffset == null) {
            log("no note detect, aborting");
            return true;
        }
        if (arm.isNoteDetected()) {
            log("note detect, finishing");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        lighting.removePattern(IntakeWithVision.class);
        swerve.stop();
    }

}

