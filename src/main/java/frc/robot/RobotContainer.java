// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.Constants.LightingConstants.VISPOSE;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.climb.DefaultClimbCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.swervedrive.TeleopDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
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

    private final HughVisionSubsystem    hugh          = new HughVisionSubsystem();
    private final JackmanVisionSubsystem jackman       = new JackmanVisionSubsystem();
    private final LightingSubsystem      lighting      = new LightingSubsystem(SIGNAL, VISPOSE);
    private final ArmSubsystem           arm           = new ArmSubsystem(lighting);
    private final ClimbSubsystem         climb         = new ClimbSubsystem(lighting);
    private final File                   yagslConfig   = new File(Filesystem.getDeployDirectory(), "swerve/neo");
    private final SwerveSubsystem        drive         = new YagslSubsystem(yagslConfig, hugh,
        lighting);
//    private final SwerveSubsystem        drive   = new RunnymedeSwerveSubsystem(hughVisionSubsystem,
//        lightingSubsystem);

    private final OperatorInput          operatorInput = new OperatorInput(
        OiConstants.DRIVER_CONTROLLER_PORT, OiConstants.OPERATOR_CONTROLLER_PORT,
        drive, arm, climb, hugh, jackman, lighting);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        drive.setDefaultCommand(new TeleopDriveCommand(drive, lighting, operatorInput));
        arm.setDefaultCommand(new DefaultArmCommand(operatorInput, arm, jackman));
        climb.setDefaultCommand(new DefaultClimbCommand(operatorInput, climb));

        operatorInput.configureTriggerBindings();
        operatorInput.initAutoSelectors();
    }

    public Command getAutonomousCommand() {
        return operatorInput.getAutonomousCommand();
    }

}