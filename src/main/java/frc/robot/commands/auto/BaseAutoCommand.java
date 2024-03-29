package frc.robot.commands.auto;

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

import static frc.robot.Constants.FieldConstants.*;

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

    protected Command intake() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new StartIntakeCommand(armSubsystem, lighting);
    }

    protected Command shoot() {
        if (Robot.isSimulation()) {
            return new WaitCommand(0.5);
        }
        return new ShootSpeakerFromAnywhereCommand(armSubsystem, swerve, lighting);
    }

    protected Command scoreSpeaker() {
        return faceSpeaker().andThen(shoot()).andThen(compact());
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

    protected Command driveRobotOriented(double vX, double vY, double omega, double seconds) {
        return new SimpleDriveRobotOrientedCommand(swerve, vX, vY, omega, seconds);
    }

    protected Command driveToNote(double spd) {
        if (Robot.isSimulation()) {
            return new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 1.35);
        }
        return new DriveToNoteCommand(swerve, lighting, armSubsystem, jackman, spd);
    }

    protected Command driveTo(Pose2d blue, Pose2d red) {
        return new DriveToPositionCommand(swerve, blue, red);
    }
}
