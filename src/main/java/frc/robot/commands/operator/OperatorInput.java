package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final XboxController driverController;
    private final XboxController operatorController;

    public enum Stick {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    /**
     * Construct an OperatorInput class that is fed by a DriverController and an
     * OperatorController.
     *
     * @param driverControllerPort on the driver station which the driver joystick
     * is plugged into
     * @param operatorControllerPort on the driver station which the aux joystick is
     * plugged into
     */
    public OperatorInput(int driverControllerPort, int operatorControllerPort) {
        driverController   = new RunnymedeGameController(driverControllerPort);
        operatorController = new RunnymedeGameController(operatorControllerPort);
    }

    public boolean isToggleTestMode() {
        return !DriverStation.isFMSAttached() && driverController.getBackButton() && driverController.getStartButton();
    }

    public XboxController getRawDriverController() {
        return driverController;
    }


    public boolean isDriverLeftBumper() {
        return driverController.getLeftBumper();
    }

    public boolean isDriverRightBumper() {
        return driverController.getRightBumper();
    }

    // Testing purposes only
    public boolean isA() {
        return driverController.getAButton();
    }

    public boolean isFaceSpeaker() {
        return driverController.getYButton();
    }

    public boolean isX() {
        return driverController.getXButton();
    }

    public boolean isB() {
        return false;
    }

    public boolean isLock() {
        return driverController.getXButton();
    }

    public boolean isZeroGyro() {
        return driverController.getBackButton();
    }

    public boolean isCancel() {
        return driverController.getStartButton();
    }

    public int getPOV() {
        return driverController.getPOV();
    }

    public double getDriverControllerAxis(Stick stick, Axis axis) {

        switch (stick) {

        case LEFT:
            switch (axis) {
            case X:
                return driverController.getLeftX();
            case Y:
                return driverController.getLeftY();
            }
            break;

        case RIGHT:
            switch (axis) {
            case X:
                return driverController.getRightX();
            }
            break;
        }

        return 0;
    }

    private boolean isIntakePressed() {
        return driverController.getAButton();
    }

    private boolean isShootPressed() {
        return driverController.getBButton();
    }

    /**
     * Set all of the robot button bindings to commands in one place
     */
    public void setButtonBindings(ArmSubsystem armSubsystem) {

        new Trigger(() -> isShootPressed())
            .onTrue(new ShootCommand(armSubsystem));

        new Trigger(() -> isIntakePressed())
            .onTrue(new StartIntakeCommand(armSubsystem));

    }

}
