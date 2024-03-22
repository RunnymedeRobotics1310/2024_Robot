package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.swervedrive.SimpleDriveRobotOrientedCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TheDoubleDown extends SequentialCommandGroup {

    public TheDoubleDown(SwerveSubsystem swerve, ArmSubsystem armSubsystem, LightingSubsystem lighting, double delay) {

        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new WaitCommand(delay));

        // score loaded
        addCommands(new ShootCommand(armSubsystem, lighting));

        // score barnum
        addCommands(new StartIntakeCommand(armSubsystem, lighting)
            .deadlineWith(new SimpleDriveRobotOrientedCommand(swerve, 1, 0, 0, 3)));
        addCommands(new CompactCommand(armSubsystem));
        addCommands(new SimpleDriveRobotOrientedCommand(swerve, -1, 0, 0, 3));
        addCommands(new ShootCommand(armSubsystem, lighting));

        // end
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
