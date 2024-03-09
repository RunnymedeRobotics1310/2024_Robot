package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;


/**
 * This command is used to safely stop the robot in its current position, and to cancel any running
 * commands
 */
public class CancelCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final OperatorInput   operatorInput;
    private final ArmSubsystem    armSubsystem;
    private final ClimbSubsystem  climbSubsystem;

    /**
     * Cancel the commands running on all subsystems.
     *
     * All subsystems must be passed to this command, and each subsystem should have a stop command
     * that safely stops the robot from moving.
     */
    public CancelCommand(OperatorInput operatorInput, SwerveSubsystem swerve, ArmSubsystem armSubsystem,
        ClimbSubsystem climbSubsystem) {

        this.swerve         = swerve;
        this.operatorInput  = operatorInput;
        this.armSubsystem   = armSubsystem;
        this.climbSubsystem = climbSubsystem;

        addRequirements(swerve);
        addRequirements(armSubsystem);
        addRequirements(climbSubsystem);

    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        /*
         * The Cancel command is not interruptable and only ends when the cancel button is released.
         */
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {

        logCommandStart();

        stopAll();
    }


    @Override
    public void execute() {
        stopAll();
    }

    @Override
    public boolean isFinished() {

        // The cancel command has a minimum timeout of .5 seconds
        if (!isTimeoutExceeded(.5)) {
            return false;
        }

        // Only end once the cancel button is released after .5 seconds has elapsed
        if (!operatorInput.isCancel()) {
            setFinishReason("Cancel button released");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

    private void stopAll() {

        // Stop all of the robot movement
        swerve.stop();
        armSubsystem.stop();
        climbSubsystem.stop();
    }
}
