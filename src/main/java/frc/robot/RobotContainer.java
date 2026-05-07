// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.swervedrive.TeleopDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.LightingConstants.*;


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

    private final LightingSubsystem      lighting      = new LightingSubsystem(
        SIGNAL_LEFT, VISPOSE_LEFT, SIGNAL_CENTER, VISPOSE_RIGHT, SIGNAL_RIGHT);
    private final ArmSubsystem           arm           = new ArmSubsystem();
    private final File                   yagslConfig   = new File(Filesystem.getDeployDirectory(), "swerve/neo");
    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem(Constants.Swerve.SUBSYSTEM_CONFIG);

    private final OperatorInput          operatorInput = new OperatorInput(
        OiConstants.DRIVER_CONTROLLER_PORT, swerveDriveSubsystem, arm, lighting);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        swerveDriveSubsystem.setDefaultCommand(new TeleopDriveCommand(swerveDriveSubsystem, lighting, operatorInput));
        arm.setDefaultCommand(new DefaultArmCommand(operatorInput, arm));

        operatorInput.configureTriggerBindings();
    }

}