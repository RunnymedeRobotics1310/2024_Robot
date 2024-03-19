package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class DriveToNoteCommand extends BaseDriveCommand {
    private final JackmanVisionSubsystem jackman;
    private final ArmSubsystem           arm;
    private long                         intakeStartTime;
    private boolean                      startTimeSet = false;
    private final double                 speedMPS;

    private boolean                      finished     = false;



    // todo: fixme: specify unit in speed param name (e.g. speedRPM, speedDegPerSec, etc.)
    public DriveToNoteCommand(SwerveSubsystem drive, ArmSubsystem arm, JackmanVisionSubsystem jackman, double speedMPS) {

        super(drive);
        this.jackman  = jackman;
        this.speedMPS = speedMPS;
        this.arm      = arm;
        addRequirements(jackman);
    }

    public void initialize() {
        logCommandStart();
    }

    public void execute() {
        super.execute();

        Rotation2d robotRelativeOffset = jackman.getNoteOffset();

        if (robotRelativeOffset == null) {
            log("no note detect, aborting");
            finished = true;
        }
        if (arm.isNoteDetected()) {
            log("note detect, finishing");
            finished = true;
        }

        if (finished) {
            return;
        }

        double setSpeed = speedMPS;

        if (Math.abs(robotRelativeOffset.getDegrees()) > 10) {
            setSpeed = 0;
        }

        Rotation2d omega = computeOmega(robotRelativeOffset);
        swerve.driveRobotOriented(new ChassisSpeeds(setSpeed, 0, omega.getRadians()));


    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}

