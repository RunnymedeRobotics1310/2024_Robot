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
        MOVE_TO_SPEAKER, MOVE_TO_UNLOCK, MOVE_TO_OVER_BUMPER, SET_SHOOTER_SPEED, IS_FINISHED
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
            System.out.println("No note detected in robot. AimSpeakerCommand cancelled");
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

        if (armSubsystem.getAimAngle() < ArmConstants.UNLOCK_POSITION.aimAngle) {
            state = State.MOVE_TO_UNLOCK;
        }
        else if (armSubsystem.getLinkAngle() < ArmConstants.OVER_BUMPER_POSITION.linkAngle) {
            state = State.MOVE_TO_OVER_BUMPER;
        }
        else {
            state = State.MOVE_TO_SPEAKER;
        }
    }

    @Override
    public boolean isFinished() {
        return (state == State.IS_FINISHED);
    }

    @Override
    public void execute() {

        final boolean atArmAngle;

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
                    gDSSA.getDegrees() - ArmConstants.SHOOTER_AIM_DIFFERENCE, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
                    ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);
            }


            if (atArmAngle) {
                logStateTransition("Start Shooter", "Arm at Shooter Position");
                state = State.MOVE_TO_SPEAKER;
            }
            break;

        case MOVE_TO_OVER_BUMPER:

            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.OVER_BUMPER_POSITION, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
                ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

            // If past the bumper danger, move to the speaker position.

            // If the aim is higher than the over-the-bumper angle, then it is safe to start
            // raising the link to the speaker position.
            if (atArmAngle) {
                logStateTransition("Move to Speaker", "Arm over bumper");
                state = State.MOVE_TO_SPEAKER;
            }

            break;

        case MOVE_TO_UNLOCK:

            // Move to the requested angle with a tolerance of 5 deg
            atArmAngle = this.driveToArmPosition(ArmConstants.UNLOCK_POSITION, ArmConstants.DEFAULT_LINK_TOLERANCE_DEG,
                ArmConstants.DEFAULT_AIM_TOLERANCE_DEG);

            // If past the bumper danger, move to the compact position.
            if (atArmAngle) {
                logStateTransition("Move to over bumper", "Arm at unlock position");
                state = State.MOVE_TO_OVER_BUMPER;
            }

        case SET_SHOOTER_SPEED:
            armSubsystem.setShooterSpeed(ArmConstants.SHOOTER_SPEAKER_SPEED);

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