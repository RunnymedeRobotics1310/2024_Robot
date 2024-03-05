package frc.robot.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.SystemTestCommand;
import frc.robot.commands.arm.AimAmpCommand;
import frc.robot.commands.arm.AimSpeakerCommand;
import frc.robot.commands.arm.CompactPoseCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.JackmanVisionSubsystem;

/**
 * The Operator input class is used to map buttons to functions and functions to commands
 * <p>
 * This class extends SubsystemBase so that the periodic() routine is called each loop. The periodic
 * routine can be used to send debug information to the dashboard
 */
public class OperatorInput extends SubsystemBase {

    public final GameController driverController   = new GameController(
        OperatorConstants.DRIVER_CONTROLLER_PORT,
        OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);

    public final GameController operatorController = new GameController(
        OperatorConstants.OPERATOR_CONTROLLER_PORT,
        OperatorConstants.GAME_CONTROLLER_STICK_DEADBAND);

    // Allow the system test command to access the controller directly
    public GameController getDriverController() {
        return driverController;
    }

    public GameController getOperatorController() {
        return operatorController;
    }

    // Either controller should be able to cancel a command
    // NOTE: When the back button is held, the cancel (start) button is used to start test mode.
    public boolean isCancelPressed() {
        return driverController.getStartButton() && !driverController.getBackButton()
            || operatorController.getStartButton() && !operatorController.getBackButton();
    }

    /**
     * Use this method to define your button -> command mappings.
     *
     * NOTE: all subsystems should be passed into this method.
     */
    public void configureButtonBindings(ArmSubsystem armSubsystem,
        JackmanVisionSubsystem visionSubsystem, ClimbSubsystem climbSubsystem) {

        // System Test - only when FMS is not attached!
        new Trigger(() -> driverController.getStartButton() && driverController.getBackButton() && !DriverStation.isFMSAttached())
            .onTrue(new SystemTestCommand(this, armSubsystem));

        // Cancel
        new Trigger(() -> isCancelPressed())
            .onTrue(new CancelCommand(this, armSubsystem, climbSubsystem));

        // Compact
        new Trigger(() -> driverController.getXButton())
            .onTrue(new CompactPoseCommand(armSubsystem));

        // Start Intake
        new Trigger(() -> driverController.getAButton())
            .onTrue(new StartIntakeCommand(armSubsystem));

        // Aim Amp
        new Trigger(() -> driverController.getBButton())
            .onTrue(new AimAmpCommand(armSubsystem));

        // Aim Speaker
        new Trigger(() -> driverController.getYButton())
            .onTrue(new AimSpeakerCommand(armSubsystem));

        // Shoot
        new Trigger(() -> driverController.getRightTriggerAxis() > .4)
            .onTrue(new ShootCommand(armSubsystem));

    }

    @Override
    public void periodic() {

        // Display any operator input values on the smart dashboard.

        SmartDashboard.putString("Driver Controller", driverController.toString());
    }

}