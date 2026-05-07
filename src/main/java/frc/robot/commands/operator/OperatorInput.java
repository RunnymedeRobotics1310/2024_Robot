package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.arm.ReverseNoteCommand;
import frc.robot.commands.arm.ShootCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Enabled;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final SwerveSubsystem                                      drive;
    private final ArmSubsystem                                         arm;
    private final LightingSubsystem                                    lighting;
    private final XboxController                                       driverController;

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
     */
    public OperatorInput(int driverControllerPort, SwerveSubsystem drive, ArmSubsystem arm, LightingSubsystem lighting) {
        this.drive         = drive;
        this.arm           = arm;
        this.lighting      = lighting;

        driverController   = new RunnymedeGameController(driverControllerPort);
    }

    public XboxController getRawDriverController() {
        return driverController;
    }

    public int getDriverPOV() {
        return driverController.getPOV();
    }

    public boolean isDriverLeftBumper() {
        return driverController.getLeftBumperButton();
    }

    public boolean isDriverRightBumper() {
        return driverController.getRightBumperButton();
    }

    public boolean isDriveFacingSpeaker() {
        return false;
    }

    public boolean isDriveFacingChain() {
        return false;
    }

    public boolean isCancel() {
        return (driverController.getStartButton());
    }

    public boolean isShift() {
        return driverController.getRightBumperButton();
    }

    /**
     * Get the aim adjustment.
     * This method will be called every 20ms (50Hz)
     *
     * @return 20ms adjustment
     */
    public double getAimAdjust() {

        double degreeAdjust = 10.0 / 50.0; // 10 deg/sec / 50 Hz

        if (driverController.getPOV() == 90) {
            return degreeAdjust;
        }

        // pov left/right for aim adjust
        if (driverController.getPOV() == 270) {
            return -degreeAdjust;
        }

        return 0;
    }

    /**
     * Get the link adjustment.
     * This method will be called every 20ms (50Hz)
     *
     * @return 20ms adjustment
     */
    public double getLinkAdjust() {

        double degreeAdjust = 10.0 / 50.0; // 10 deg/sec / 50 Hz

        if (driverController.getPOV() == 0) {
            return degreeAdjust;
        }

        // pov left/right for aim adjust
        if (driverController.getPOV() == 180) {
            return -degreeAdjust;
        }

        return 0;
    }

    public double getDriverControllerAxis(Stick stick, Axis axis) {

        return switch (stick) {
        case LEFT -> switch (axis) {
        case X -> driverController.getLeftX();
        case Y -> driverController.getLeftY();
        };
        case RIGHT -> switch (axis) {
        case X -> driverController.getRightX();
        case Y -> driverController.getRightY();
        };
        };

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox} /
     * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    public void configureTriggerBindings() {

        //
        // GLOBAL & COMMON COMMANDS
        //

        // Run when enabled
        new Trigger(RobotController::isSysActive)
            .onTrue(new InstantCommand(() -> lighting.addSignalPattern(Enabled.getInstance())));



        //
        // DRIVER CONTROLLER BINDINGS
        //

        // human intake
        new Trigger(() -> (driverController.getLeftTriggerAxis() > 0.5/* || driverController.getRightTriggerAxis() > 0.5*/))
            .onTrue(new ReverseNoteCommand(arm));

        // start intake
        new Trigger(() -> driverController.getRightTriggerAxis() > 0.5)
            .onTrue(new StartIntakeCommand(arm, lighting));

        // zero gyro
        new Trigger(driverController::getBackButton).onTrue(new ZeroGyroCommand(drive));

        // cancel command (driver)
        new Trigger(this::isCancel).whileTrue(new CancelCommand(this, drive, arm));


        // shoot
        new Trigger(() -> driverController.getXButton() && isShift())
            .onTrue(new ShootCommand(1, arm, lighting));

        new Trigger(() -> driverController.getXButton() && !isShift())
            .onTrue(new ShootCommand(0.2, arm, lighting));

        // close shoot
        new Trigger(driverController::getYButton)
            .onTrue(new ShootCommand(0.5, arm, lighting));

        // mid shoot
        new Trigger(driverController::getBButton)
            .onTrue(new ShootCommand(0.4, arm, lighting));

        // Violet friendly shoot
        new Trigger(driverController::getAButton)
            .onTrue(new ShootCommand(0.3, arm, lighting));



        //
        // OPERATOR CONTROLLER BINDINGS
        //

        // cancel command (operator)
        new Trigger(this::isCancel).whileTrue(new CancelCommand(this, drive, arm));

    }

}