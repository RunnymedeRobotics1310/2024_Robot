package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
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
    private long                         intakeStartTime;
    private boolean                      startTimeSet = false;
    private final double                 speedMPS;

    private boolean                      finished     = false;



    // todo: fixme: specify unit in speed param name (e.g. speedRPM, speedDegPerSec, etc.)
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
        finished = false;
    }


    @Override
    public void execute() {
        super.execute();

        Rotation2d robotRelativeOffset = jackman.getNoteOffset();
        finished = false;

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
        log("finished: " + finished);
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        lighting.removePattern(IntakeWithVision.class);

        if (interrupted) {
            log("interrupted!");
        }

    }

}

