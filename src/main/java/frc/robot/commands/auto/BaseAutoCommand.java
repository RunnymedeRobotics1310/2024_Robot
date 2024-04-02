package frc.robot.commands.auto;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.UsefulPoses.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
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

    Command compactCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new CompactCommand(armSubsystem);
    }

    private Command compactFromIntakeCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new CompactFromIntakeCommand(armSubsystem, false);
    }

    private Command startIntakeCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new StartIntakeCommand(armSubsystem, lighting);
    }

    private Command reverseNoteCommand() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new ReverseNoteCommand(armSubsystem);
    }

    private Command shootFromAnywhereCommand() {
        return new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting);
    }

    protected Command scoreSpeaker() {
        Command drive   = faceSpeakerCommand();
        Command prePrep = compactFromIntakeCommand().alongWith(reverseNoteCommand());
        if (Robot.isSimulation()) {
            return drive.alongWith(prePrep).andThen(new WaitCommand(0.5));
        }
        return drive.alongWith(prePrep.andThen(shootFromAnywhereCommand()));
    }

    private Command faceSpeakerCommand() {
        return RotateToTargetCommand.createRotateToSpeakerCommand(swerve);
    }

    private Command faceWolverineCommand(Rotation2d tolerance) {
        return new RotateToLocationCommand(swerve, BLUE_WOLVERINE, RED_WOLVERINE, tolerance);
    }

    private Command faceBarnumCommand(Rotation2d tolerance) {
        return new RotateToLocationCommand(swerve, BLUE_BARNUM, RED_BARNUM, tolerance);
    }

    private Command faceValjeanCommand(Rotation2d tolerance) {
        return new RotateToLocationCommand(swerve, BLUE_VALJEAN, RED_VALJEAN, tolerance);
    }

    private Command faceCenterNoteCommand(Rotation2d tolerance) {
        return new RotateToLocationCommand(swerve, CENTRE_NOTE_3, CENTRE_NOTE_3, tolerance);
    }

    private Command driveRobotOriented(double xSpeedMps, double ySpeedMps, double omegaRadPerSec, double seconds) {
        return new SimpleDriveRobotOrientedCommand(swerve, xSpeedMps, ySpeedMps, omegaRadPerSec, seconds);
    }

    private Command driveToNoteCommand(double speedMps) {
        if (Robot.isSimulation()) {
            return new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 0.2).andThen(wait(0.5));
        }
        return new DriveToNoteCommand(swerve, lighting, armSubsystem, jackman, speedMps);
    }

    protected Command driveTo(Pose2d blue, Pose2d red) {
        return new DriveToPositionCommand(swerve, blue, red, Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS);
    }

    protected Command approach(Translation2d blue, Translation2d red, double separationMetres) {
        return new ApproachPositionCommand(swerve, blue, red, Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS,
            separationMetres);
    }

    protected Command goGetWolverine() {
        Command arm   = wait(0.25).andThen(startIntakeCommand());
        // get away from speaker before rotating
        Command drive = driveTo(IN_FRONT_OF_WOLVERINE_BLUE, IN_FRONT_OF_WOLVERINE_RED)
//            .andThen(faceWolverineCommand())
            .andThen(approach(BLUE_WOLVERINE, RED_WOLVERINE, 0.8))
            .andThen(driveToNoteCommand(2));
        return arm.alongWith(drive);
//        return arm;
    }


    protected Command goGetBarnum() {
        Command arm   = compactCommand().andThen(startIntakeCommand());
        Command drive = faceBarnumCommand(Rotation2d.fromDegrees(15))                                                     /*
                                                                                                                           * .andThen
                                                                                                                           * (
                                                                                                                           * approach
                                                                                                                           * (
                                                                                                                           * BLUE_BARNUM,
                                                                                                                           * RED_BARNUM,
                                                                                                                           * 0
                                                                                                                           * .
                                                                                                                           * 80
                                                                                                                           * )
                                                                                                                           * )
                                                                                                                           */
            .andThen(driveToNoteCommand(2));
        return arm.alongWith(drive);
    }

    protected Command goGetValjean() {
        Command arm   = compactCommand().andThen(startIntakeCommand());
        Command drive = faceValjeanCommand(Rotation2d.fromDegrees(15))                                                       /*
                                                                                                                              * .andThen
                                                                                                                              * (
                                                                                                                              * approach
                                                                                                                              * (
                                                                                                                              * BLUE_VALJEAN,
                                                                                                                              * RED_VALJEAN,
                                                                                                                              * 0
                                                                                                                              * .
                                                                                                                              * 80
                                                                                                                              * )
                                                                                                                              * )
                                                                                                                              */
            .andThen(driveToNoteCommand(2));
        return arm.alongWith(drive);
    }

    protected Command goGetCenterNote() {
        Command part1 = compactCommand().alongWith(approach(UNDER_STAGE_NEAR_BARNUM_BLUE, UNDER_STAGE_NEAR_BARNUM_RED, 0.30)
            .andThen(approach(UNDER_STAGE_BLUE, UNDER_STAGE_RED, 0.30)));
        Command arm   = wait(2.0).andThen(startIntakeCommand());
        Command drive = approach(CENTRE_NOTE_3, CENTRE_NOTE_3, 0.80).andThen(driveToNoteCommand(2));
        return part1.andThen(arm.alongWith(drive));
    }

    protected Command goGetNote5() {
        Command arm   = wait(2.0).andThen(startIntakeCommand());
        Command drive = approach(CENTRE_NOTE_5, CENTRE_NOTE_5, 0.80).andThen(driveToNoteCommand(2));
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

    protected Command exitSourceSide() {
        return driveTo(AFTER_WOLVERINE_AUTO_BLUE, AFTER_WOLVERINE_AUTO_RED)
            .andThen(driveTo(PARK_AFTER_WOLVERINE_AUTO_BLUE, PARK_AFTER_WOLVERINE_AUTO_RED));
    }

    protected Command exitMiddle() {
        return driveTo(PARK_AFTER_BARNUM_AUTO_BLUE, PARK_AFTER_BARNUM_AUTO_RED);
    }

    protected Command exitAmpSide() {
        return driveTo(PARK_AFTER_VALJEAN_AUTO_BLUE, PARK_AFTER_VALJEAN_AUTO_RED);
    }

}
