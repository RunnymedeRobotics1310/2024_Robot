package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RunnymedeUtils;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

/**
 * Move arm to speaker shoot pose
 * Set shooter speed (distance based)
 */
public class AimSpeakerCommand extends ArmBaseCommand {

    private enum State {
        MOVE_TO_SPEAKER, SET_SHOOTER_SPEED, FIRE_NOTE, FIRING_NOW, IS_FINISHED
    };

    private State                     state = State.MOVE_TO_SPEAKER;

    private final HughVisionSubsystem hughVisionSubsystem;

    public AimSpeakerCommand(ArmSubsystem armSubsystem, HughVisionSubsystem hughVisionSubsystem) {

        super(armSubsystem);
        this.hughVisionSubsystem = hughVisionSubsystem;
    }

    @Override
    public void initialize() {

        // If there is no note detected, then why are we aiming?
        if (!armSubsystem.isNoteDetected()) {
            log("No note detected in robot. AimSpeakerCommand cancelled");
            state = State.IS_FINISHED;
            return;
        }

        logCommandStart();

        DriverStation.Alliance alliance = RunnymedeUtils.getRunnymedeAlliance();
        if (alliance == DriverStation.Alliance.Blue) {
            hughVisionSubsystem.setBotTarget(Constants.BotTarget.BLUE_SPEAKER);
        }
        else {
            hughVisionSubsystem.setBotTarget(Constants.BotTarget.RED_SPEAKER);
        }

//        if (armSubsystem.getAimAngle() < ArmConstants.UNLOCK_POSITION.aimAngle) {
//            state = State.MOVE_TO_UNLOCK;
//        }
//        else if (armSubsystem.getLinkAngle() < ArmConstants.OVER_BUMPER_POSITION.linkAngle) {
//            state = State.MOVE_TO_OVER_BUMPER;
//        }
//        else {
//            state = State.MOVE_TO_SPEAKER;
//        }
    }

    @Override
    public boolean isFinished() {
        return (state == State.IS_FINISHED);
    }

    @Override
    public void execute() {

        boolean atArmAngle;

        switch (state) {

        case MOVE_TO_SPEAKER:
            Translation2d getShooterXY = armSubsystem.getShooterXY();

            // todo: fixme: give a hint to units - method name, variable name, param name, or else
            // return Rotation2d
            Rotation2d gDSSA = hughVisionSubsystem.getDynamicSpeakerShooterAngle(getShooterXY);

            if (gDSSA == null) {
                atArmAngle = this.driveToArmPosition(ArmConstants.SHOOT_SPEAKER_ARM_POSITION,
                    ArmConstants.DEFAULT_LINK_TOLERANCE_DEG, ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);
            }
            else {
                atArmAngle = this.driveToArmPosition(ArmConstants.SHOOT_SPEAKER_ARM_POSITION.linkAngle,
                    -gDSSA.getDegrees() + 90, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
                    ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);
            }

            // shooter facing up == 0
            // shooter moving down == -angle
            //
            // getDynamicSpeakerShooterAngle == 0 == horizontal
            // angle moving up == +angle

            if (atArmAngle) {
                logStateTransition("Start Shooter", "Arm at Shooter Position");
                state = State.SET_SHOOTER_SPEED;
            }
            break;

        case SET_SHOOTER_SPEED:
            armSubsystem.setShooterSpeed(ArmConstants.SHOOTER_SPEAKER_SPEED);
            logStateTransition(State.FIRE_NOTE.name(), "Shooter motor started");
            break;

        case FIRE_NOTE:
            if (armSubsystem.getShooterEncoderSpeed() >= 1000) {
                armSubsystem.setIntakeSpeed(0.75);
                logStateTransition(State.FIRING_NOW.name(), "Intake Stated, let's gooo!");
                state = State.FIRING_NOW;
            }
            break;

        case FIRING_NOW:
            if (armSubsystem.getIntakeEncoderSpeed() >= 1000) {
                armSubsystem.setIntakeSpeed(0);
                armSubsystem.setShooterSpeed(0);
                logStateTransition(State.IS_FINISHED.name(), "We've fired");
                state = State.IS_FINISHED;
            }
            break;

        case IS_FINISHED:
            break;

        }
    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();
        hughVisionSubsystem.setBotTarget(Constants.BotTarget.ALL);

        logCommandEnd(interrupted);
    }

}