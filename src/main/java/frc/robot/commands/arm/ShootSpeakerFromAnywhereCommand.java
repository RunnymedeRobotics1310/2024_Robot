package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Shooting;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.telemetry.Light;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

/**
 * Move arm to speaker shoot pose
 * Set shooter speed (distance based)
 */
public class ShootSpeakerFromAnywhereCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_UNLOCK, REVERSE_NOTE, START_SHOOTER, START_FEEDER, FINISHED
    };

    private SwerveSubsystem     swerveSubsystem;
    private HughVisionSubsystem hughVisionSubsystem;
    private LightingSubsystem   lighting;

    private State               state               = State.MOVE_TO_UNLOCK;
    double                      intakeStartPosition = 0;
    long                        shooterStartTime    = 0;

    public ShootSpeakerFromAnywhereCommand(ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem,
        HughVisionSubsystem hughVisionSubsystem, LightingSubsystem lighting) {
        super(armSubsystem);
        this.swerveSubsystem     = swerveSubsystem;
        this.hughVisionSubsystem = hughVisionSubsystem;
        this.lighting            = lighting;
        addRequirements(swerveSubsystem, hughVisionSubsystem);
    }

    @Override
    public void initialize() {
        lighting.addPattern(SIGNAL, Shooting.getInstance());
        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            log("No note detected in robot. AimSpeakerCommand cancelled");
            state = State.FINISHED;
            return;
        }

        logCommandStart();

        if (isAtArmPosition(ArmConstants.COMPACT_ARM_POSITION, 2)) {
            state = State.MOVE_TO_UNLOCK;
        }
        else {
            state = State.REVERSE_NOTE;
        }

        intakeStartPosition = armSubsystem.getIntakePosition();

        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            hughVisionSubsystem.setBotTarget(Constants.BotTarget.BLUE_SPEAKER);
        }
        else {
            hughVisionSubsystem.setBotTarget(Constants.BotTarget.RED_SPEAKER);
        }
    }

    @Override
    public void execute() {

        final boolean atArmAngle;

        switch (state) {

        case MOVE_TO_UNLOCK:

            // Run the link motor back (up) for .15 seconds to unlock the arm
            armSubsystem.setLinkPivotSpeed(.3);
            armSubsystem.setAimPivotSpeed(0);

            if (isStateTimeoutExceeded(.2)) {
                logStateTransition("Unlock -> Move To Speaker", "Arm Unlocked");
                state = State.REVERSE_NOTE;
            }

            break;

        case REVERSE_NOTE:

            armSubsystem.setShooterSpeed(-0.1);
            armSubsystem.setIntakeSpeed(-0.3);

            // Reverse the note for a number of rotations
            if (Math.abs(armSubsystem.getIntakePosition() - intakeStartPosition) > 2) {
                logStateTransition("Reverse -> Start Shooter", "Shooter Reversed");
                state = State.START_SHOOTER;
            }

            break;

        case START_SHOOTER:

            // Drive to the arm position at the same time
            double linkAngle = ArmConstants.SHOOT_SPEAKER_PODIUM_ARM_POSITION.linkAngle;
            // TODO: Supply real arm position
            Rotation2d shooterAngle = hughVisionSubsystem.getDynamicSpeakerShooterAngle(new Translation2d(0, 0));
            // TODO: Replace this default 45 angle with a field location based one
            double angle = 45;
            if (shooterAngle != null) {
                angle = shooterAngle.getDegrees();
            }
            double aimAngle = angle - (linkAngle - 180);
            Constants.ArmPosition armPositionNew = new Constants.ArmPosition(linkAngle, aimAngle);

            atArmAngle = this.driveToArmPosition(armPositionNew,
                ArmConstants.DEFAULT_LINK_TOLERANCE_DEG, ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

            armSubsystem.setIntakeSpeed(0);
            armSubsystem.setShooterSpeed(.75);

            // Wait for the shooter to get up to speed and the arm to get into position
            if (isStateTimeoutExceeded(.75) && atArmAngle) {
                logStateTransition("Start Shooter -> Shoot", "Shooter up to speed " + armSubsystem.getShooterEncoderSpeed());
                state = State.START_FEEDER;
            }

            break;

        case START_FEEDER:

            // Continue to drive to the arm position while shooting
            atArmAngle = this.driveToArmPosition(ArmConstants.SHOOT_SPEAKER_PODIUM_ARM_POSITION,
                ArmConstants.DEFAULT_LINK_TOLERANCE_DEG, ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

            armSubsystem.setIntakeSpeed(1);

            if (isStateTimeoutExceeded(.5)) {
                logStateTransition("Shoot -> Finished", "Shot fired");
                state = State.FINISHED;
            }
            break;

        case FINISHED:

            break;

        }
    }

    @Override
    public boolean isFinished() {

        if (state == State.FINISHED) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        lighting.removePattern(Shooting.class);

        armSubsystem.setAimPivotSpeed(0);
        armSubsystem.setLinkPivotSpeed(0);

        armSubsystem.setIntakeSpeed(0);
        armSubsystem.setShooterSpeed(0);

        hughVisionSubsystem.setBotTarget(Constants.BotTarget.ALL);

        logCommandEnd(interrupted);

        if (!interrupted) {
            if (DriverStation.isTeleop()) {
//                CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
            }

        }

    }

}