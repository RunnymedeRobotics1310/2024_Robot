package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.*;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.UsefulPoses.*;
import static frc.robot.Constants.UsefulPoses.PARK_AFTER_WOLVERINE_AUTO_RED;

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

    protected Command compactCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new CompactCommand(armSubsystem);
    }

    protected Command armToPointBCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        // TODO: replace compact with a new command that goes to Point B instead (above bumper)
        return new CompactCommand(armSubsystem);
    }

    protected Command compactFromIntakeCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new CompactFromIntakeCommand(armSubsystem, false);
    }

    protected Command startIntakeCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new StartIntakeCommand(armSubsystem, lighting);
    }


    protected Command shootSpeakerPrepCommand() {
        // todo: implement : spins up shooter and aims arm to speaker
        return new InstantCommand();
    }

    protected Command fireCommand() {
        // todo: implement
        return new InstantCommand();
    }

    protected Command reverseNoteCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new ReverseNoteCommand(armSubsystem);
    }

    protected Command scoreSpeaker() {
        Command drive   = faceSpeakerCommand();
        Command prePrep = armToPointBCommand().alongWith(reverseNoteCommand());
        Command prep    = shootSpeakerPrepCommand();
        Command fire    = fireCommand();
        // todo: uncomment when fixes are implemented
        // return drive.alongWith(prePrep.andThen(prep)).andThen(fire);
        return scoreSpeakerBackupPlan();
    }

    protected Command scoreSpeakerBackupPlan() {
        Command drive   = faceSpeakerCommand();
        Command prePrep = armToPointBCommand().alongWith(reverseNoteCommand());
        if (Robot.isSimulation()) {
            return drive.alongWith(prePrep).andThen(new WaitCommand(0.5));
        }
        return drive.alongWith(prePrep).andThen(new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting));
    }

    protected Command faceSpeakerCommand() {
        return RotateToTargetCommand.createRotateToSpeakerCommand(swerve);
    }

    protected Command faceBarnumCommand() {
        return new RotateToLocationCommand(swerve, BLUE_BARNUM, RED_BARNUM);
    }

    protected Command faceValjeanCommand() {
        return new RotateToLocationCommand(swerve, BLUE_VALJEAN, RED_VALJEAN);
    }

    protected Command driveRobotOriented(double xSpeedMps, double ySpeedMps, double omegaRadPerSec, double seconds) {
        return new SimpleDriveRobotOrientedCommand(swerve, xSpeedMps, ySpeedMps, omegaRadPerSec, seconds);
    }

    protected Command driveToNoteCommand(double speedMps) {
        if (Robot.isSimulation()) {
            return new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 1.35);
        }
        return new DriveToNoteCommand(swerve, lighting, armSubsystem, jackman, speedMps);
    }

    protected Command driveTo(Pose2d blue, Pose2d red) {
        return new DriveToPositionCommand(swerve, blue, red);
    }

    protected Command goGetWolverine() {
        Command arm   = armToPointBCommand().andThen(startIntakeCommand());
        Command drive = driveTo(IN_FRONT_OF_WOLVERINE_BLUE, IN_FRONT_OF_WOLVERINE_RED).andThen(driveToNoteCommand(2));
        return arm.alongWith(drive);
    }


    protected Command goGetBarnum() {
        Command arm   = armToPointBCommand().andThen(startIntakeCommand());
        Command drive = faceBarnumCommand().andThen(driveToNoteCommand(2));
        return arm.alongWith(drive);
    }

    protected Command goGetValjean() {
        Command arm   = armToPointBCommand().andThen(startIntakeCommand());
        Command drive = faceValjeanCommand().andThen(driveToNoteCommand(2));
        return arm.alongWith(drive);
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
        return driveToAmp().andThen(aimAmp()).andThen(new ShootCommand(armSubsystem, lighting)).andThen(compactCommand());
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
