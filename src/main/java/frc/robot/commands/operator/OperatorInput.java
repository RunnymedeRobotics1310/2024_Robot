package frc.robot.commands.operator;

import static frc.robot.Constants.UsefulPoses.SCORE_BLUE_AMP;

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
import frc.robot.commands.arm.CompactPoseCommand;
import frc.robot.commands.arm.IntakeEjectCommand;
import frc.robot.commands.arm.IntakeSimpleCommand;
import frc.robot.commands.arm.ManualShootSimpleCommand;
import frc.robot.commands.arm.ShShShakeItOffCommand;
import frc.robot.commands.arm.StartIntakeCommand2;
import frc.robot.commands.auto.PlanBAutoCommand;
import frc.robot.commands.auto.Score1AmpAutoCommand;
import frc.robot.commands.auto.Score1SpeakerAutoCommand;
import frc.robot.commands.auto.Score2AmpAutoCommand;
import frc.robot.commands.auto.Score2SpeakerAutoCommand;
import frc.robot.commands.auto.Score2_5AmpAutoCommand;
import frc.robot.commands.auto.Score3SpeakerAutoCommand;
import frc.robot.commands.auto.Score4SpeakerAutoCommand;
import frc.robot.commands.swervedrive.DriveToNoteCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
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

    public boolean isShakeItOff() {
        return isShift() && operatorController.getAButton();
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
        // new
        // Trigger(driverController::getBButton).onTrue(RotateToTargetCommand.createRotateToSpeakerCommand(drive,
        // hugh));

        // Compact
        new Trigger(() -> driverController.getXButton() || operatorController.getXButton()).onTrue(new CompactPoseCommand(arm));

        // Start Intake
        new Trigger(() -> operatorController.getPOV() == 270).onTrue(new IntakeSimpleCommand(arm, jackman));

        // Vision note pickup
        new Trigger(driverController::getBButton).onTrue(new StartIntakeCommand2(arm, jackman)
            .alongWith(new DriveToNoteCommand(drive, arm, jackman, 0.25)));

        // Aim Amp
        // new Trigger(operatorController::getAButton).onTrue(new AimAmpCommand(arm));

        // Aim Speaker
        // new Trigger(operatorController::getYButton).onTrue(new AimSpeakerCommand(arm, hugh));

        // Shoot
        new Trigger(operatorController::getBButton).onTrue((new ManualShootSimpleCommand(arm)));

        new Trigger(() -> operatorController.getPOV() == 90).whileTrue(new IntakeEjectCommand(arm));

        new Trigger(this::isShakeItOff).whileTrue(new ShShShakeItOffCommand(arm));

        // Test Drive to 2,2,20
        // new Trigger(driverController::getXButton).onTrue(new DriveToPositionCommand(drive,
        // BLUE_2_2_20, RED_2_2_20));

        // Climbs Up pov 0
        // Climbs Down pov 180
        // Trap pov 90

        // setpose for practice fields

        // in front of speaker
        new Trigger(() -> operatorController.getPOV() == 0)
            .onTrue(new ResetOdometryCommand(drive,
                new Pose2d(Constants.BotTarget.BLUE_SPEAKER.getLocation().getX(), 1.6, new Rotation2d()),
                new Pose2d(Constants.BotTarget.RED_SPEAKER.getLocation().getX(), 16.54 - 1.6, new Rotation2d())));

        // in front of amp
        new Trigger(() -> operatorController.getPOV() == 180)
            .onTrue(new ResetOdometryCommand(drive, SCORE_BLUE_AMP, Constants.UsefulPoses.SCORE_RED_AMP));

    }

    public void initAutoSelectors() {

        Telemetry.auto.autoPatternChooser = autoPatternChooser;

        autoPatternChooser.setDefaultOption("Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);

        autoPatternChooser.addOption("1 Amp", Constants.AutoConstants.AutoPattern.SCORE_1_AMP);
        autoPatternChooser.addOption("2 Amp", Constants.AutoConstants.AutoPattern.SCORE_2_AMP);
        autoPatternChooser.addOption("2.5 Amp", Constants.AutoConstants.AutoPattern.SCORE_2_5_AMP);

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
        case SCORE_1_AMP -> new Score1AmpAutoCommand(drive, hugh);
        case SCORE_2_AMP -> new Score2AmpAutoCommand(drive, arm, hugh, jackman);
        case SCORE_2_5_AMP -> new Score2_5AmpAutoCommand(drive, arm, hugh, jackman);
        case SCORE_1_SPEAKER -> new Score1SpeakerAutoCommand(drive, arm, hugh);
        case SCORE_2_SPEAKER -> new Score2SpeakerAutoCommand(drive, arm, hugh, jackman);
        case SCORE_3_SPEAKER -> new Score3SpeakerAutoCommand(drive, arm, hugh, jackman);
        case SCORE_4_SPEAKER -> new Score4SpeakerAutoCommand(drive, arm, hugh, jackman);
        case PLAN_B -> new PlanBAutoCommand(drive);
        default -> new InstantCommand();
        };
    }

}
