package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class DriveToScoreAmpCommand extends BaseDriveCommand {

    private Pose2d nearby;
    private Pose2d amp;
    private double speed;

    private enum State {
        MOVE_NEARBY, CLOSE_ALIGNING, DONE
    }

    private State state = State.MOVE_NEARBY;

    public DriveToScoreAmpCommand(SwerveSubsystem swerve) {
        super(swerve);
        this.speed = Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
    }

    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            this.nearby = Constants.UsefulPoses.PRE_SCORE_BLUE_AMP;
            this.amp    = Constants.UsefulPoses.SCORE_BLUE_AMP;
        }
        else {
            this.nearby = Constants.UsefulPoses.PRE_SCORE_RED_AMP;
            this.amp    = Constants.UsefulPoses.SCORE_RED_AMP;
        }
        if (isCloseEnough(nearby)) {
            state = State.CLOSE_ALIGNING;
        }
        else {
            state = State.MOVE_NEARBY;
        }
    }

    @Override
    public void execute() {
        super.execute();

        switch (state) {
        case MOVE_NEARBY:
            driveToFieldPose(nearby, speed);
            if (isCloseEnough(nearby)) {
                state = State.CLOSE_ALIGNING;
            }
            break;
        case CLOSE_ALIGNING:
            driveToFieldPose(amp, speed / 2);
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return isCloseEnough(amp);
    }
}