package frc.robot.commands.operator;

import static frc.robot.Constants.UsefulPoses.BLUE_2_2_20;
import static frc.robot.Constants.UsefulPoses.RED_2_2_20;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.arm.*;
import frc.robot.commands.auto.PlanBAutoCommand;
import frc.robot.commands.auto.Score1AmpAutoCommand;
import frc.robot.commands.auto.Score1SpeakerAutoCommand;
import frc.robot.commands.auto.Score2AmpAutoCommand;
import frc.robot.commands.auto.Score2_5AmpAutoCommand;
import frc.robot.commands.auto.Score3SpeakerAutoCommand;
import frc.robot.commands.auto.Score4SpeakerAutoCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Enabled;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.telemetry.Telemetry;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final SwerveSubsystem                                      drive;
    private final ArmSubsystem                                         arm;
    private final LightingSubsystem                                    lighting;
    private final HughVisionSubsystem                                  hugh;
    private final ClimbSubsystem                                       climb;
    private final JackmanVisionSubsystem                               jackman;
    private final XboxController                                       driverController;
    private final XboxController                                       operatorController;

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
    public OperatorInput(int driverControllerPort, int operatorControllerPort, SwerveSubsystem drive, ArmSubsystem arm,
        ClimbSubsystem climb, HughVisionSubsystem hugh, JackmanVisionSubsystem jackman, LightingSubsystem lighting) {
        this.drive         = drive;

        this.arm           = arm;
        this.lighting      = lighting;
        this.hugh          = hugh;
        this.climb         = climb;
        this.jackman       = jackman;

        driverController   = new RunnymedeGameController(driverControllerPort);
        operatorController = new RunnymedeGameController(operatorControllerPort);
    }

    public XboxController getRawDriverController() {
        return driverController;
    }

    public int getDriverPOV() {
        return driverController.getPOV();
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
        return (driverController.getStartButton() || operatorController.getStartButton());
    }

    public boolean isShift() {
        return operatorController.getRightBumper();
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

    public double getOperatorControllerAxis(Stick stick, Axis axis) {

        return switch (stick) {
        case LEFT -> switch (axis) {
        case X -> operatorController.getLeftX();
        case Y -> operatorController.getLeftY();
        };
        case RIGHT -> switch (axis) {
        case X -> operatorController.getRightX();
        case Y -> operatorController.getRightY();
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

        // Run when enabled
        new Trigger(RobotController::isSysActive).onTrue(new InstantCommand(() -> lighting.addPattern(Enabled.getInstance())));

        // Activate test mode
        new Trigger(() -> !DriverStation.isFMSAttached() && driverController.getBackButton() && driverController.getStartButton())
            .onTrue(new SystemTestCommand(this, drive, arm, climb, lighting));

        // Cancel all systems
        new Trigger(this::isCancel).whileTrue(new CancelCommand(this, drive, arm, climb));

        // zero gyro
        new Trigger(driverController::getBackButton).onTrue(new ZeroGyroCommand(drive));

        // Rotate to speaker
        new Trigger(driverController::getBButton).onTrue(RotateToTargetCommand.createRotateToSpeakerCommand(drive, hugh));

        // Compact
        new Trigger(() -> driverController.getXButton() || operatorController.getXButton()).onTrue(new CompactPoseCommand(arm));

        // Start Intake
        new Trigger(driverController::getAButton).onTrue(new StartIntakeCommand(arm)); // IntakeCommand(arm, jackman);

        // Aim Amp
        new Trigger(operatorController::getAButton).onTrue(new AimAmpCommand(arm));

        // Aim Speaker
        new Trigger(operatorController::getYButton).onTrue(new AimSpeakerCommand(arm, hugh));

        // Shoot
        new Trigger(operatorController::getBButton).onTrue(new ManualShootCommand(arm));

        // Test Drive to 2,2,20
        new Trigger(driverController::getXButton).onTrue(new DriveToPositionCommand(drive, BLUE_2_2_20, RED_2_2_20));

        // Climbs Up pov 0
        // Climbs Down pov 180
        // Climbs Down pov 270
        // Trap pov 90
        // Shift to Climb right bumper

    }

    public void initAutoSelectors() {

        Telemetry.auto.autoPatternChooser = autoPatternChooser;

        autoPatternChooser.setDefaultOption("Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);

        autoPatternChooser.addOption("1 Amp", Constants.AutoConstants.AutoPattern.SCORE_1_AMP);
        autoPatternChooser.addOption("2 Amp", Constants.AutoConstants.AutoPattern.SCORE_2_AMP);
        autoPatternChooser.addOption("2.5 Amp", Constants.AutoConstants.AutoPattern.SCORE_2_5_AMP);

        autoPatternChooser.addOption("1 Speaker", Constants.AutoConstants.AutoPattern.SCORE_1_SPEAKER);
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
        case SCORE_1_AMP -> new Score1AmpAutoCommand(drive, hugh);
        case SCORE_2_AMP -> new Score2AmpAutoCommand(drive, arm, hugh, jackman);
        case SCORE_2_5_AMP -> new Score2_5AmpAutoCommand(drive, arm, hugh, jackman);
        case SCORE_1_SPEAKER -> new Score1SpeakerAutoCommand(drive, hugh);
        case SCORE_3_SPEAKER -> new Score3SpeakerAutoCommand(drive, arm, hugh, jackman);
        case SCORE_4_SPEAKER -> new Score4SpeakerAutoCommand(drive, arm, hugh, jackman);
        case PLAN_B -> new PlanBAutoCommand(drive);
        default -> new InstantCommand();
        };
    }

}
