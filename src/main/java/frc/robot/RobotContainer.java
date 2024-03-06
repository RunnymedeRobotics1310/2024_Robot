// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.Constants.LightingConstants.VISPOSE;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.arm.AimAmpCommand;
import frc.robot.commands.arm.AimSpeakerCommand;
import frc.robot.commands.arm.CompactPoseCommand;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.climb.DefaultClimbCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.commands.swervedrive.TeleopDriveCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Enabled;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.yagsl.YagslSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...

    private final HughVisionSubsystem    hugh               = new HughVisionSubsystem();
    private final JackmanVisionSubsystem jackman            = new JackmanVisionSubsystem();
    private final LightingSubsystem      lighting           = new LightingSubsystem(SIGNAL, VISPOSE);
    private final ArmSubsystem           arm                = new ArmSubsystem(lighting);
    private final ClimbSubsystem         climb              = new ClimbSubsystem(lighting);

    // todo: set up sendable chooser for this to toggle implementation for testing
    private final File                   yagslConfig        = new File(Filesystem.getDeployDirectory(), "swerve/neo");
    private final SwerveSubsystem        drive              = new YagslSubsystem(yagslConfig, hugh,
        lighting);
//    private final SwerveSubsystem        drive   = new RunnymedeSwerveSubsystem(hughVisionSubsystem,
//        lightingSubsystem);

    SendableChooser<AutoPattern>         autoPatternChooser = new SendableChooser<>();

    private final OperatorInput          operatorInput      = new OperatorInput(
        OiConstants.DRIVER_CONTROLLER_PORT, OiConstants.OPERATOR_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        drive.setDefaultCommand(new TeleopDriveCommand(drive, lighting, operatorInput));
        arm.setDefaultCommand(new DefaultArmCommand(operatorInput, arm));
        climb.setDefaultCommand(new DefaultClimbCommand(operatorInput, climb));

        configureTriggerBindings();
        initAutoSelectors();
    }

    private void initAutoSelectors() {

        // FIXME: (low) consider moving all of the choosers to their own classes.
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        autoPatternChooser.addOption("1 Amp", AutoPattern.SCORE_1_AMP);
        autoPatternChooser.addOption("2 Amp", AutoPattern.SCORE_2_AMP);
        autoPatternChooser.addOption("1 Speaker", AutoPattern.SCORE_1_SPEAKER);
        autoPatternChooser.addOption("3 Speaker", AutoPattern.SCORE_3_SPEAKER);
        autoPatternChooser.addOption("4 Speaker", AutoPattern.SCORE_4_SPEAKER);
        autoPatternChooser.addOption("Plan B", AutoPattern.PLAN_B);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);
        autoPatternChooser.addOption("Three Note", AutoPattern.THREE_NOTE);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox} /
     * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureTriggerBindings() {

        // Run when enabled
        new Trigger(RobotController::isSysActive).onTrue(new InstantCommand(() -> lighting.addPattern(Enabled.getInstance())));

        // Activate test mode
        new Trigger(operatorInput::isToggleTestMode).onTrue(new SystemTestCommand(operatorInput, drive, arm, lighting));

        new Trigger(operatorInput::isZeroGyro).onTrue(new ZeroGyroCommand(drive));
        new Trigger(operatorInput::isCancel).whileTrue(new CancelCommand(operatorInput, drive, arm, climb));
        new Trigger(operatorInput::isB).onTrue(RotateToTargetCommand.createRotateToSpeakerCommand(drive, hugh));

        // Compact
        new Trigger(operatorInput::isCompactPressed).onTrue(new CompactPoseCommand(arm));

        // Start Intake
        new Trigger(operatorInput::isStartIntake).onTrue(new StartIntakeCommand(arm));

        // Aim Amp
        new Trigger(operatorInput::isAimAmp).onTrue(new AimAmpCommand(arm));

        // Aim Speaker
        new Trigger(operatorInput::isAimSpeaker).onTrue(new AimSpeakerCommand(arm, hugh));

        // Shoot
        new Trigger(operatorInput::isShoot).onTrue(new ShootCommand(arm));

        // Climbs Up pov 0
        // Climbs Down pov 180
        // Climbs Down pov 270
        // Trap pov 90
        // Shift to Climb right bumper

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return switch (autoPatternChooser.getSelected()) {
        case SCORE_1_AMP -> new Score1AmpAutoCommand(drive, hugh);
        case SCORE_2_AMP -> new Score2AmpAutoCommand(drive, arm, hugh, jackman);
        case SCORE_2_5_AMP -> new Score2_5AmpAutoCommand(drive, arm, hugh, jackman);
        case SCORE_1_SPEAKER -> new Score1SpeakerAutoCommand(drive, hugh);
        case SCORE_3_SPEAKER -> new Score3SpeakerAutoCommand(drive, arm, hugh, jackman);
        case SCORE_4_SPEAKER -> new Score4SpeakerAutoCommand(drive, arm, hugh, jackman);
        case PLAN_B -> new PlanBAutoCommand(drive);
        // If the chooser did not work, then do nothing as the default auto.
        default -> new InstantCommand();
        };

    }

}