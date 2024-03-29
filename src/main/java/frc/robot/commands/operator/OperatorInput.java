package frc.robot.commands.operator;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.Constants.UsefulPoses.SCORE_BLUE_AMP;
import static frc.robot.Constants.UsefulPoses.SCORE_RED_AMP;
import static frc.robot.Constants.UsefulPoses.START_AT_BLUE_SPEAKER;
import static frc.robot.Constants.UsefulPoses.START_AT_RED_SPEAKER;

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
import frc.robot.commands.auto.Score1AmpAutoCommand;
import frc.robot.commands.auto.Score1SpeakerAutoCommand;
import frc.robot.commands.auto.Score1SpeakerStayAutoCommand;
import frc.robot.commands.auto.Score2AmpAutoCommand;
import frc.robot.commands.auto.Score2_5AmpAutoCommand;
import frc.robot.commands.auto.Score4SpeakerAutoCommand;
import frc.robot.commands.auto.ScoreLoadedBarnumValjean;
import frc.robot.commands.auto.ScoreLoadedWolverineBarnumValjean;
import frc.robot.commands.auto.TheDoubleDown;
import frc.robot.commands.climb.MaxClimbCommand;
import frc.robot.commands.swervedrive.DriveToNoteCommand;
import frc.robot.commands.swervedrive.DriveToScoreAmpCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Enabled;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.JackmanVisionSubsystem;
import frc.robot.telemetry.Telemetry;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final SwerveSubsystem                                              drive;
    private final ArmSubsystem                                                 arm;
    private final LightingSubsystem                                            lighting;
    private final ClimbSubsystem                                               climb;
    private final JackmanVisionSubsystem                                       jackman;
    private final XboxController                                               driverController;
    private final XboxController                                               operatorController;

    private final SendableChooser<Constants.AutoConstants.AutoPattern>         autoPatternChooser               = new SendableChooser<>();
    private final SendableChooser<Constants.AutoConstants.Delay>               delayChooser                     = new SendableChooser<>();
    private final SendableChooser<Constants.ArmConstants.TrapShootMotorSpeeds> trapShootTopMotorSpeedChooser    = new SendableChooser<>();
    private final SendableChooser<Constants.ArmConstants.TrapShootMotorSpeeds> trapShootBottomMotorSpeedChooser = new SendableChooser<>();

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
        ClimbSubsystem climb, JackmanVisionSubsystem jackman, LightingSubsystem lighting) {
        this.drive         = drive;

        this.arm           = arm;
        this.lighting      = lighting;
        this.climb         = climb;
        this.jackman       = jackman;

        driverController   = new RunnymedeGameController(driverControllerPort);
        operatorController = new RunnymedeGameController(operatorControllerPort);
    }

    public XboxController getRawDriverController() {
        return driverController;
    }

    public XboxController getRawOperatorController() {
        return operatorController;
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



        //
        // DRIVER CONTROLLER BINDINGS
        //

        // vision note pickup
        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
            .onTrue(new StartIntakeCommand(arm, lighting)
                .deadlineWith(new DriveToNoteCommand(drive, lighting, arm, jackman, 2)));

        // start intake
        new Trigger(() -> driverController.getRightTriggerAxis() > 0.5)
            .onTrue(new StartIntakeCommand(arm, lighting));

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
        new Trigger(driverController::getXButton).onTrue(new CompactFromIntakeCommand(arm, true));



        //
        // OPERATOR CONTROLLER BINDINGS
        //

        // cancel command (operator)
        new Trigger(this::isCancel).whileTrue(new CancelCommand(this, drive, arm, climb));

        // Trap
        new Trigger(() -> operatorController.getBackButton() && operatorController.getYButton())
            .onTrue(new ShootTrapCommand(arm, climb));

        new Trigger(() -> this.isShift() && operatorController.getXButton())
            .onTrue(new ShootTrapFromFloorCommand(arm, lighting, this));

        // rotate aim shoot
        new Trigger(() -> !this.isShift() && operatorController.getAButton())
            .onTrue(new ShootSpeakerFromAnywhereMcMullinStyleCommand(arm, drive, lighting));

        // rotate aim shoot
        new Trigger(() -> !this.isShift() && operatorController.getXButton())
            .onTrue(new ShootSpeakerFromAnywhereCommand(arm, drive, lighting));

        // podium shot
        new Trigger(() -> !operatorController.getBackButton() && operatorController.getYButton())
            .onTrue(new ShootSpeakerFromPodiumCommand(arm, lighting));

        // shoot FIRE
        // IF YOU CHANGE THE BUTTON THIS IS ON, MUST CHANGE THE BUTTON RELEASE
        // IN ShootPrepFireCommand as well.
        new Trigger(() -> !this.isShift() && operatorController.getBButton())
            .onTrue(new ShootPrepFireCommand(arm, lighting, this));

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
        autoPatternChooser.addOption("The Double Down", Constants.AutoConstants.AutoPattern.THE_DOUBLE_DOWN);
        autoPatternChooser.addOption("Loaded + Wolverine (Speaker)", Constants.AutoConstants.AutoPattern.SCORE_LOADED_WOLVERINE);
        autoPatternChooser.addOption("2 Speaker Vision", Constants.AutoConstants.AutoPattern.SCORE_2_SPEAKER_VISION);
        autoPatternChooser.addOption("3 Speaker", Constants.AutoConstants.AutoPattern.SCORE_3_SPEAKER);
        autoPatternChooser.addOption("Loaded + Wolverine + Barnum (Speaker)",
            Constants.AutoConstants.AutoPattern.SCORE_LOADED_WOLVERINE_BARNUM);
        autoPatternChooser.addOption("Loaded + Wolverine + Barnum + Valjean (Speaker)",
            Constants.AutoConstants.AutoPattern.SCORE_LOADED_WOLVERINE_BARNUM_VALJEAN);
        autoPatternChooser.addOption("Loaded + Barnum + Valjean (Speaker)",
            Constants.AutoConstants.AutoPattern.SCORE_LOADED_BARNUM_VALJEAN);
        autoPatternChooser.addOption("4 Speaker", Constants.AutoConstants.AutoPattern.SCORE_4_SPEAKER);



        Telemetry.auto.delayChooser = delayChooser;

        delayChooser.setDefaultOption("No Delay", Constants.AutoConstants.Delay.NO_DELAY);
        delayChooser.addOption("1/2 Seconds", Constants.AutoConstants.Delay.WAIT_0_5_SECOND);
        delayChooser.addOption("1 Second", Constants.AutoConstants.Delay.WAIT_1_SECOND);
        delayChooser.addOption("1 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_1_5_SECONDS);
        delayChooser.addOption("2 Seconds", Constants.AutoConstants.Delay.WAIT_2_SECONDS);
        delayChooser.addOption("2 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_2_5_SECONDS);
        delayChooser.addOption("3 Seconds", Constants.AutoConstants.Delay.WAIT_3_SECONDS);
        delayChooser.addOption("5 Seconds", Constants.AutoConstants.Delay.WAIT_5_SECONDS);
    }


    public void initTrapShooterSpeedSelectors() {

        Telemetry.arm.trapShootTopMotorSpeedChooser = trapShootTopMotorSpeedChooser;

        trapShootTopMotorSpeedChooser.setDefaultOption("0.3", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_THREE);

        trapShootTopMotorSpeedChooser.addOption("0.1", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_ONE);
        trapShootTopMotorSpeedChooser.addOption("0.15", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_ONE5);
        trapShootTopMotorSpeedChooser.addOption("0.2", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_TWO);
        trapShootTopMotorSpeedChooser.addOption("0.25", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_TWO5);
        trapShootTopMotorSpeedChooser.addOption("0.35", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_THREE5);
        trapShootTopMotorSpeedChooser.addOption("0.4", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_FOUR);
        trapShootTopMotorSpeedChooser.addOption("0.45", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_FOUR5);
        trapShootTopMotorSpeedChooser.addOption("0.5", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_FIVE);
        trapShootTopMotorSpeedChooser.addOption("0.55", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_FIVE5);
        trapShootTopMotorSpeedChooser.addOption("0.6", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_SIX);
        trapShootTopMotorSpeedChooser.addOption("0.65", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_SIX5);
        trapShootTopMotorSpeedChooser.addOption("0.7", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_SEVEN);
        trapShootTopMotorSpeedChooser.addOption("0.8", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_EIGHT);
        trapShootTopMotorSpeedChooser.addOption("0.9", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_NINE);
        trapShootTopMotorSpeedChooser.addOption("1.0", Constants.ArmConstants.TrapShootMotorSpeeds.ONE);

        Telemetry.arm.trapShootBottomMotorSpeedChooser = trapShootBottomMotorSpeedChooser;

        trapShootBottomMotorSpeedChooser.setDefaultOption("0.4", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_FOUR);

        trapShootBottomMotorSpeedChooser.addOption("0.1", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_ONE);
        trapShootBottomMotorSpeedChooser.addOption("0.15", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_ONE5);
        trapShootBottomMotorSpeedChooser.addOption("0.2", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_TWO);
        trapShootBottomMotorSpeedChooser.addOption("0.25", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_TWO5);
        trapShootBottomMotorSpeedChooser.addOption("0.3", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_THREE);
        trapShootBottomMotorSpeedChooser.addOption("0.35", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_THREE5);
        trapShootBottomMotorSpeedChooser.addOption("0.45", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_FOUR5);
        trapShootBottomMotorSpeedChooser.addOption("0.5", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_FIVE);
        trapShootBottomMotorSpeedChooser.addOption("0.55", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_FIVE5);
        trapShootBottomMotorSpeedChooser.addOption("0.6", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_SIX);
        trapShootBottomMotorSpeedChooser.addOption("0.65", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_SIX5);
        trapShootBottomMotorSpeedChooser.addOption("0.7", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_SEVEN);
        trapShootBottomMotorSpeedChooser.addOption("0.8", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_EIGHT);
        trapShootBottomMotorSpeedChooser.addOption("0.9", Constants.ArmConstants.TrapShootMotorSpeeds.ZERO_NINE);
        trapShootBottomMotorSpeedChooser.addOption("1.0", Constants.ArmConstants.TrapShootMotorSpeeds.ONE);
    }

    private double getTrapShootMotorSpeed(
        SendableChooser<Constants.ArmConstants.TrapShootMotorSpeeds> trapShootMotorSpeedChooser) {

        return switch (trapShootMotorSpeedChooser.getSelected()) {
        case ZERO_ONE -> 0.1;
        case ZERO_ONE5 -> 0.15;
        case ZERO_TWO -> 0.2;
        case ZERO_TWO5 -> 0.25;
        case ZERO_THREE -> 0.3;
        case ZERO_THREE5 -> 0.35;
        case ZERO_FOUR -> 0.4;
        case ZERO_FOUR5 -> 0.45;
        case ZERO_FIVE -> 0.5;
        case ZERO_FIVE5 -> 0.55;
        case ZERO_SIX -> 0.6;
        case ZERO_SIX5 -> 0.65;
        case ZERO_SEVEN -> 0.7;
        case ZERO_EIGHT -> 0.8;
        case ZERO_NINE -> 0.9;
        case ONE -> 1;
        };
    }

    public double getTrapShootTopMotorSpeed() {
        return getTrapShootMotorSpeed(trapShootTopMotorSpeedChooser);
    }

    public double getTrapShootBottomMotorSpeed() {
        return getTrapShootMotorSpeed(trapShootBottomMotorSpeedChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        double delay = switch (delayChooser.getSelected()) {
        case WAIT_0_5_SECOND -> 0.5;
        case WAIT_1_SECOND -> 1;
        case WAIT_1_5_SECONDS -> 1.5;
        case WAIT_2_SECONDS -> 2;
        case WAIT_2_5_SECONDS -> 2.5;
        case WAIT_3_SECONDS -> 3;
        case WAIT_5_SECONDS -> 5;
        default -> 0;
        };

        return switch (autoPatternChooser.getSelected()) {
        // not used
        case SCORE_1_AMP -> new Score1AmpAutoCommand(drive, arm, lighting, delay);
        case SCORE_2_AMP -> new Score2AmpAutoCommand(drive, arm, jackman, lighting, delay);
        case SCORE_2_5_AMP -> new Score2_5AmpAutoCommand(drive, arm, jackman, lighting, delay);
        case SCORE_2_SPEAKER_VISION -> new Score4SpeakerAutoCommand(drive, arm, jackman, lighting, delay, 2);
        case SCORE_3_SPEAKER -> new Score4SpeakerAutoCommand(drive, arm, jackman, lighting, delay, 3);
        case SCORE_4_SPEAKER -> new Score4SpeakerAutoCommand(drive, arm, jackman, lighting, delay, 4);

        // used in competition
        case EXIT_ZONE -> new ExitZoneAutoCommand(drive, delay);
        case SCORE_1_SPEAKER_STAY -> new Score1SpeakerStayAutoCommand(drive, arm, lighting, delay);
        case SCORE_1_SPEAKER -> new Score1SpeakerAutoCommand(drive, arm, lighting, delay);
        case THE_DOUBLE_DOWN -> new TheDoubleDown(drive, arm, lighting, delay);
        case SCORE_LOADED_WOLVERINE -> new ScoreLoadedWolverineBarnumValjean(drive, arm, jackman, lighting, delay, 2);
        case SCORE_LOADED_WOLVERINE_BARNUM -> new ScoreLoadedWolverineBarnumValjean(drive, arm, jackman, lighting, delay, 3);
        case SCORE_LOADED_WOLVERINE_BARNUM_VALJEAN ->
            new ScoreLoadedWolverineBarnumValjean(drive, arm, jackman, lighting, delay, 4);
        case SCORE_LOADED_BARNUM_VALJEAN -> new ScoreLoadedBarnumValjean(drive, arm, jackman, lighting, delay);
        default -> new InstantCommand();
        };
    }
}