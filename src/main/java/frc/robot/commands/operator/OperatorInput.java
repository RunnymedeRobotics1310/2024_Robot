package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.arm.StartIntakeCommand2;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final ArmSubsystem                                         arm;
    private final ClimbSubsystem                                       climb;
    private final XboxController                                       driverController;
//    private final XboxController                                       operatorController;

    private final SendableChooser<Constants.AutoConstants.AutoPattern> autoPatternChooser = new SendableChooser<>();

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
    public OperatorInput(int driverControllerPort, int operatorControllerPort, ArmSubsystem arm,
        ClimbSubsystem climb) {

        this.arm         = arm;
        this.climb       = climb;

        driverController = new RunnymedeGameController(driverControllerPort);
        // operatorController = new RunnymedeGameController(operatorControllerPort);
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

    public boolean isDriveFacingSpeaker() {
        return driverController.getYButton();
    }

    public boolean isCancel() {
        return (driverController.getStartButton()); // || operatorController.getStartButton());
    }

//    public boolean isShift() {
//        return operatorController.getRightBumper();
//    }
//
//    public boolean isShakeItOff() {
//        return isShift() && operatorController.getAButton();
//    }
//
//    public boolean isClimbPosition() {
//        return isShift() && operatorController.getBButton();
//    }
//
//    public boolean isManualShoot() {
//        return !isShift() && operatorController.getBButton();
//    }

    public double getLeftClimbSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    public double getRightClimbSpeed() {
        // TODO Auto-generated method stub
        return 0;
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

//    public double getOperatorControllerAxis(Stick stick, Axis axis) {
//
//        return switch (stick) {
//        case LEFT -> switch (axis) {
//        case X -> operatorController.getLeftX();
//        case Y -> operatorController.getLeftY();
//        };
//        case RIGHT -> switch (axis) {
//        case X -> operatorController.getRightX();
//        case Y -> operatorController.getRightY();
//        };
//        };
//
//    }


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

        // Activate test mode
        new Trigger(() -> !DriverStation.isFMSAttached() && driverController.getBackButton() && driverController.getStartButton())
            .onTrue(new SystemTestCommand(this, arm, climb));

        // Cancel all systems
        new Trigger(this::isCancel).onTrue(new CancelCommand(this, arm, climb));

        // Rotate to speaker
        // new
        // Trigger(driverController::getBButton).onTrue(RotateToTargetCommand.createRotateToSpeakerCommand(drive,
        // hugh));

        // Compact
        // new Trigger(() -> driverController.getXButton() ||
        // operatorController.getXButton()).onTrue(new CompactPoseCommand(arm));


        new Trigger(driverController::getAButton).onTrue(new StartIntakeCommand2(arm));

        // Aim Amp
        // new Trigger(operatorController::getAButton).onTrue(new AimAmpCommand(arm));

        // Aim Speaker
        // new Trigger(operatorController::getYButton).onTrue(new AimSpeakerCommand(arm, hugh));

        // Shoot
//        new Trigger(this::isManualShoot).onTrue((new ManualShootCommand(arm)));
//
//        new Trigger(() -> operatorController.getPOV() == 90).whileTrue(new IntakeEjectCommand(arm));
//
//        new Trigger(this::isShakeItOff).whileTrue(new ShShShakeItOffCommand(arm));

        // TODO: Uncomment AmpPositionCommand when link is fixed
//        new Trigger(this::isClimbPosition).onTrue(new MaxClimbCommand(climb)/*
//                                                                             * .alongWith(new
//                                                                             * AmpPositionCommand(
//                                                                             * arm))
//                                                                             */);
//
        // Test Drive to 2,2,20
        // new Trigger(driverController::getXButton).onTrue(new DriveToPositionCommand(drive,
        // BLUE_2_2_20, RED_2_2_20));

        // Climbs Up pov 0
        // Climbs Down pov 180
        // Trap pov 90

        // setpose for practice fields


    }

    public void initAutoSelectors() {


        autoPatternChooser.setDefaultOption("Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);

        autoPatternChooser.addOption("Exit Zone", Constants.AutoConstants.AutoPattern.EXIT_ZONE);
        autoPatternChooser.addOption("1 Amp", Constants.AutoConstants.AutoPattern.SCORE_1_AMP);
        autoPatternChooser.addOption("2 Amp", Constants.AutoConstants.AutoPattern.SCORE_2_AMP);
        autoPatternChooser.addOption("2.5 Amp", Constants.AutoConstants.AutoPattern.SCORE_2_5_AMP);

        autoPatternChooser.addOption("1 Speaker Stay", Constants.AutoConstants.AutoPattern.SCORE_1_SPEAKER_STAY);
        autoPatternChooser.addOption("1 Speaker", Constants.AutoConstants.AutoPattern.SCORE_1_SPEAKER);
        autoPatternChooser.addOption("2 Speaker", Constants.AutoConstants.AutoPattern.SCORE_2_SPEAKER);
        autoPatternChooser.addOption("3 Speaker", Constants.AutoConstants.AutoPattern.SCORE_3_SPEAKER);
        autoPatternChooser.addOption("4 Speaker", Constants.AutoConstants.AutoPattern.SCORE_4_SPEAKER);

        autoPatternChooser.addOption("Plan B", Constants.AutoConstants.AutoPattern.PLAN_B);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return switch (autoPatternChooser.getSelected()) {
        default -> new InstantCommand();
        };
    }

}
