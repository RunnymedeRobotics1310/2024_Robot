package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CancelCommand extends LoggingCommand {

    private final SwerveSubsystem      swerve;
    private static final Translation2d DONT_MOVE = new Translation2d();
    private static final Rotation2d    DONT_TURN = new Rotation2d();

    private final ArmSubsystem         armSubsystem;

    public CancelCommand(SwerveSubsystem swerve, ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;
        this.swerve       = swerve;

        addRequirements(swerve, armSubsystem);

    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {

        swerve.driveFieldOriented(DONT_MOVE, DONT_TURN);

        armSubsystem.stop();
    }
}