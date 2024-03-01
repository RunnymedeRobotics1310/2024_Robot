// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JackmanVisionSubsystem;


public class DriveToTargetCommand extends Command {

    private final double                 timeoutSeconds;
    private final double                 speed;
    private final DriveSubsystem         driveSubsystem;
    private double                       desiredHeadingDelta;
    private final JackmanVisionSubsystem visionSubsystem;

    private long                         startTimeMs = 0;

    /**
     * DriveForTime command drives at the specified heading at the specified speed for the specified
     * time.
     *
     * @param timeoutSeconds to run the command
     * @param speed in the range -1.0 to +1.0
     * @param driveSubsystem
     */
    public DriveToTargetCommand(double timeoutSeconds, double speed, DriveSubsystem driveSubsystem,
        JackmanVisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.timeoutSeconds  = timeoutSeconds;
        this.speed           = speed;
        this.driveSubsystem  = driveSubsystem;


        // Add required subsystems
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTimeMs = System.currentTimeMillis();
        SmartDashboard.putNumber("heading", driveSubsystem.getHeading() + visionSubsystem.getTargetX());
        System.out.println("Command initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        desiredHeadingDelta = visionSubsystem.getTargetX();
        double errorF = desiredHeadingDelta * 0.02;
        driveSubsystem.setMotorSpeeds(speed + errorF, speed - errorF);

        SmartDashboard.putNumber("Delta", desiredHeadingDelta);
        System.out.println("Command executed");
        // Nothing to do here except wait for the end
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // When the command finishes, do nothing
        // NOTE: control will return to the driver
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        double runTimeSeconds = (System.currentTimeMillis() - startTimeMs) / 1000.0d;

        if (runTimeSeconds >= timeoutSeconds) {
            System.out.println("Command finished");
            return true;
        }

        return false;
    }
}
