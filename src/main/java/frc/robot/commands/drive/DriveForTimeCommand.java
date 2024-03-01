// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * An example command that uses the drive subsystem.
 */
public class DriveForTimeCommand extends Command {

    private final double         timeoutSeconds;
    private final double         speed;
    private final DriveSubsystem driveSubsystem;

    private long                 startTimeMs = 0;

    /**
     * DriveForTime command drives straight at the specified speed for the specified time.
     *
     * @param timeoutSeconds to run the command
     * @param speed in the range -1.0 to +1.0
     * @param driveSubsystem
     */
    public DriveForTimeCommand(double timeoutSeconds, double speed, DriveSubsystem driveSubsystem) {

        this.timeoutSeconds = timeoutSeconds;
        this.speed          = speed;
        this.driveSubsystem = driveSubsystem;

        // Add required subsystems
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        startTimeMs = System.currentTimeMillis();
        driveSubsystem.setMotorSpeeds(speed, speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
            return true;
        }

        return false;
    }
}
