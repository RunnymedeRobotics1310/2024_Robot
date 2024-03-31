package frc.robot.commands.auto;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.UsefulPoses.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.*;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

public class BaseAutoCommand extends SequentialCommandGroup {

    private final SwerveSubsystem        swerve;
    private final ArmSubsystem           armSubsystem;
    private final JackmanVisionSubsystem jackman;
    private final LightingSubsystem      lighting;

    public BaseAutoCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem, JackmanVisionSubsystem jackman,
        LightingSubsystem lighting) {
        this.swerve       = swerve;
        this.armSubsystem = armSubsystem;
        this.jackman      = jackman;
        this.lighting     = lighting;
    }

    protected Command log(String message) {
        return new LogMessageCommand(message);
    }

    protected Command wait(double seconds) {
        return new WaitCommand(seconds);
    }

    protected Command compact() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new CompactCommand(armSubsystem);
    }

    protected Command compactFromIntake() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new CompactFromIntakeCommand(armSubsystem, false);
    }

    protected Command intake() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new StartIntakeCommand(armSubsystem, lighting)
            .andThen(compactFromIntake().alongWith(new ReverseNoteCommand(armSubsystem)));
    }

    protected Command shoot() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting, true);
    }

    protected Command scoreSpeaker() {
        return shoot().andThen(compact());
    }

    protected Command faceSpeaker() {
        return RotateToTargetCommand.createRotateToSpeakerCommand(swerve);
    }

    protected Command faceBarnum() {
        return new RotateToLocationCommand(swerve, BLUE_BARNUM, RED_BARNUM);
    }

    protected Command faceValjean() {
        return new RotateToLocationCommand(swerve, BLUE_VALJEAN, RED_VALJEAN);
    }

    protected Command driveRobotOriented(double xSpeedMps, double ySpeedMps, double omegaRadPerSec, double seconds) {
        return new SimpleDriveRobotOrientedCommand(swerve, xSpeedMps, ySpeedMps, omegaRadPerSec, seconds);
    }

    protected Command driveToNote(double speedMps) {
        if (Robot.isSimulation()) {
            return new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 1.35);
        }
        return new DriveToNoteCommand(swerve, lighting, armSubsystem, jackman, speedMps);
    }

    protected Command driveTo(Pose2d blue, Pose2d red) {
        return new DriveToPositionCommand(swerve, blue, red);
    }

    protected Command goGetWolverine() {
        return wait(0.25).andThen(intake())
            .alongWith(
                driveTo(IN_FRONT_OF_WOLVERINE_BLUE, IN_FRONT_OF_WOLVERINE_RED)
                    .andThen(driveToNote(1)));
    }


    protected Command goGetBarnum() {
        return intake()
            .alongWith(
                faceBarnum()
                    .andThen(driveToNote(1)));
    }

    protected Command goGetValjean() {
        return intake()
            .alongWith(
                faceValjean()
                    .andThen(driveToNote(1)));
    }

    protected Command driveToAmp() {
        return new DriveToScoreAmpCommand(swerve);
    }

    protected Command aimAmp() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new AimAmpCommand(armSubsystem);
    }

    protected Command scoreAmp() {
        if (Robot.isSimulation()) {
            return driveToAmp().andThen(wait(0.5));
        }
        return driveToAmp().andThen(aimAmp()).andThen(shoot()).andThen(compact());
    }

    /*
     * SEQUENCES
     */
    protected void sequenceExitSourceSide() {
        addCommands(driveTo(AFTER_WOLVERINE_AUTO_BLUE, AFTER_WOLVERINE_AUTO_RED));
        addCommands(driveTo(PARK_AFTER_WOLVERINE_AUTO_BLUE, PARK_AFTER_WOLVERINE_AUTO_RED));
    }

    protected void sequenceExitMiddle() {
        addCommands(driveTo(PARK_AFTER_BARNUM_AUTO_BLUE, PARK_AFTER_BARNUM_AUTO_RED));
    }

    protected void sequenceExitAmpSide() {
        addCommands(driveTo(PARK_AFTER_VALJEAN_AUTO_BLUE, PARK_AFTER_VALJEAN_AUTO_RED));
    }

}
