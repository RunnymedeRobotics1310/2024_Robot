package frc.robot.commands.operator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.auto.ExitZoneAutoCommand;
import frc.robot.commands.auto.PlanBAutoCommand;
import frc.robot.commands.auto.Score1AmpAutoCommand;
import frc.robot.commands.auto.Score1SpeakerAutoCommand;
import frc.robot.commands.auto.Score1SpeakerStayAutoCommand;
import frc.robot.commands.auto.Score2AmpAutoCommand;
import frc.robot.commands.auto.Score2SpeakerAutoCommand;
import frc.robot.commands.auto.Score2_5AmpAutoCommand;
import frc.robot.commands.auto.Score3SpeakerAutoCommand;
import frc.robot.commands.auto.Score4SpeakerAutoCommand;
import frc.robot.commands.climb.MaxClimbCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Enabled;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.telemetry.Telemetry;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.Constants.UsefulPoses.*;

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
    private final SendableChooser<Constants.AutoConstants.Delay>       delayChooser       = new SendableChooser<>();
    private double                                                     delay;

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

    public boolean isOperatorLeftBumper() {
        return operatorController.getLeftBumper();
    }

    public boolean isDriveFacingSpeaker() {
        return driverController.getYButton();
    }

    public boolean isDriveFacingChain() {
        return driverController.getAButton();
    }

    public boolean isCancel() {
        return (driverController.getStartButton() || operatorController.getStartButton());
    }

    public boolean isShift() {
        return operatorController.getRightBumper();
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

        //
        // GLOBAL & COMMON COMMANDS
        //

        // Run when enabled
        new Trigger(RobotController::isSysActive)
            .onTrue(new InstantCommand(() -> lighting.addPattern(SIGNAL, Enabled.getInstance())));

        // score trap command
        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5
            && driverController.getRightTriggerAxis() > 0.5
            && operatorController.getLeftTriggerAxis() > 0.5
            && operatorController.getRightTriggerAxis() > 0.5).onTrue(new ScoreTrapCommand(arm));



        //
        // DRIVER CONTROLLER BINDINGS
        //

        // vision note pickup
        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
            .onTrue(new DriveToNoteCommand(drive, lighting, arm, jackman, Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS)
                .deadlineWith(new StartIntakeCommand(arm, lighting)));

        // start intake
        new Trigger(() -> driverController.getRightTriggerAxis() > 0.5).onTrue(new StartIntakeCommand(arm, lighting));

        // zero gyro
        new Trigger(driverController::getBackButton).onTrue(new ZeroGyroCommand(drive));

        // Activate test mode
        new Trigger(() -> !DriverStation.isFMSAttached() && driverController.getBackButton() && driverController.getStartButton())
            .onTrue(new SystemTestCommand(this, drive, arm, climb, lighting));

        // cancel command (driver)
        new Trigger(this::isCancel).whileTrue(new CancelCommand(this, drive, arm, climb));

        // align amp
        new Trigger(driverController::getBButton)
            .onTrue(new DriveToScoreAmpCommand(drive));

        // compact
        new Trigger(driverController::getXButton).onTrue(new CompactCommand(arm));



        //
        // OPERATOR CONTROLLER BINDINGS
        //

        // cancel command (operator)
        new Trigger(this::isCancel).whileTrue(new CancelCommand(this, drive, arm, climb));

        // rotate aim shoot
        new Trigger(() -> !this.isShift() && operatorController.getXButton())
            .onTrue(new ShootSpeakerFromAnywhereCommand(arm, drive, hugh, lighting));

        // podium shot
        new Trigger(operatorController::getYButton).onTrue(new ShootSpeakerFromPodiumCommand(arm, lighting));

        // shoot
        new Trigger(() -> !this.isShift() && operatorController.getBButton()).onTrue(new ShootCommand(arm, lighting));

        // set pose at speaker
        new Trigger(() -> this.isShift() && operatorController.getBButton())
            .onTrue(new ResetOdometryCommand(
                drive, START_AT_BLUE_SPEAKER, START_AT_RED_SPEAKER));

        // set pose at amp
        new Trigger(() -> this.isShift() && operatorController.getAButton())
            .onTrue(new ResetOdometryCommand(drive, SCORE_BLUE_AMP, SCORE_RED_AMP));


        // climbers up
        new Trigger(() -> this.isShift() && operatorController.getPOV() == 0)
            .onTrue(new MaxClimbCommand(climb, drive));

        // aim amp
        new Trigger(() -> operatorController.getPOV() == 270)
            .onTrue(new AimAmpCommand(arm));

        // eject
        new Trigger(() -> operatorController.getPOV() == 90)
            .whileTrue(new EjectNoteCommand(arm));

        // aim source
        new Trigger(() -> operatorController.getPOV() == 180)
            .onTrue(new AimSourceCommand(arm));

    }

    public void initAutoSelectors() {

        Telemetry.auto.autoPatternChooser = autoPatternChooser;

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


        Telemetry.auto.delayChooser = delayChooser;

        delayChooser.setDefaultOption("No Delay", Constants.AutoConstants.Delay.NO_DELAY);
        delayChooser.addOption("1/2 Seconds", Constants.AutoConstants.Delay.WAIT_0_5_SECOND);
        delayChooser.addOption("1 Second", Constants.AutoConstants.Delay.WAIT_1_SECOND);
        delayChooser.addOption("1 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_1_5_SECONDS);
        delayChooser.addOption("2 Seconds", Constants.AutoConstants.Delay.WAIT_2_SECONDS);
        delayChooser.addOption("2 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_2_5_SECONDS);
        delayChooser.addOption("3 Seconds", Constants.AutoConstants.Delay.WAIT_3_SECONDS);
        delayChooser.addOption("5 Seconds", Constants.AutoConstants.Delay.WAIT_5_SECONDS);


        delay = switch (delayChooser.getSelected()) {
        case WAIT_0_5_SECOND -> 0.5;
        case WAIT_1_SECOND -> 1;
        case WAIT_1_5_SECONDS -> 1.5;
        case WAIT_2_SECONDS -> 2;
        case WAIT_2_5_SECONDS -> 2.5;
        case WAIT_3_SECONDS -> 3;
        case WAIT_5_SECONDS -> 5;
        default -> 0;
        };
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return switch (autoPatternChooser.getSelected()) {
        case EXIT_ZONE -> new ExitZoneAutoCommand(drive, delay);
        case SCORE_1_AMP -> new Score1AmpAutoCommand(drive, arm, hugh, lighting, delay);
        case SCORE_2_AMP -> new Score2AmpAutoCommand(drive, arm, hugh, jackman, lighting, delay);
        case SCORE_2_5_AMP -> new Score2_5AmpAutoCommand(drive, arm, hugh, jackman, lighting, delay);
        case SCORE_1_SPEAKER_STAY -> new Score1SpeakerStayAutoCommand(drive, arm, hugh, lighting, delay);
        case SCORE_1_SPEAKER -> new Score1SpeakerAutoCommand(drive, arm, hugh, lighting, delay);
        case SCORE_2_SPEAKER -> new Score2SpeakerAutoCommand(drive, arm, hugh, jackman, lighting, delay);
        case SCORE_3_SPEAKER -> new Score3SpeakerAutoCommand(drive, arm, hugh, jackman, lighting, delay);
        case SCORE_4_SPEAKER -> new Score4SpeakerAutoCommand(drive, arm, hugh, jackman, lighting, delay);
        case PLAN_B -> new PlanBAutoCommand(drive, delay);
        default -> new InstantCommand();
        };
    }
}