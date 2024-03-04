// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JackmanVisionSubsystem;
import frc.robot.subsystems.LightsSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The operator input class
    private final OperatorInput          operatorInput   = new OperatorInput();

    // The robot's subsystems and commands are defined here...
    private final LightsSubsystem        lightsSubsystem = new LightsSubsystem();
    private final JackmanVisionSubsystem visionSubsystem = new JackmanVisionSubsystem();
    private final ArmSubsystem           armSubsystem    = new ArmSubsystem(lightsSubsystem);
//    private final ClimbSubsystem         climbSubsystem  = new ClimbSubsystem(lightsSubsystem);



    // All dashboard choosers are defined here...
    private final SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        armSubsystem.setDefaultCommand(
            new DefaultArmCommand(
                operatorInput, armSubsystem));

//        climbSubsystem.setDefaultCommand(
//            new DefaultClimbCommand(
//                operatorInput, climbSubsystem));

        // Initialize the dashboard choosers
        initDashboardChoosers();

        // Configure the button bindings
        operatorInput.configureButtonBindings(armSubsystem, visionSubsystem);

        // Add a trigger for the robot enabled
        new Trigger(() -> RobotController.isSysActive())
            .onTrue(
                new InstantCommand(() -> lightsSubsystem.setEnabled()));
    }

    private void initDashboardChoosers() {

        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);
        autoPatternChooser.addOption("Three Note", AutoPattern.THREE_NOTE);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // Pass in all of the subsystems and all of the choosers to the auto command.
        return new AutonomousCommand(
            autoPatternChooser);
    }
}
