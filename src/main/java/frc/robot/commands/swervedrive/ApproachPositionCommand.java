package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ApproachPositionCommand extends DriveToPositionFacingCommand {

    private final double endWhileDrivingWhenThisCloseMetres;

    /**
     * This command drives toward a specified position at the speed specified, but this command ENDS
     * when the distance between the robot and the target pose closes to the distance specified.
     * The heading is not a factor in determining when this ends.
     *
     * Note, the robot doesn't stop moving at this point - it doesn't even decelerate when it gets
     * close (though
     * it might already be decelerating because it's close to the final destination).
     *
     * This is really useful when you want to switch your drive command from drive using odometry to
     * drive using vision.
     * 
     * @param endWhileDrivingWhenThisCloseMetres the distance at which the robot is "close enough"
     * to the target pose that you want to end the command
     * @see DriveToPositionCommand
     * @see DriveToNoteCommand
     */
    public ApproachPositionCommand(SwerveSubsystem swerve, Translation2d bluePose, Translation2d redPose, double speed,
        double endWhileDrivingWhenThisCloseMetres) {
        super(swerve, bluePose, redPose, speed);
        this.endWhileDrivingWhenThisCloseMetres = endWhileDrivingWhenThisCloseMetres;
    }

    @Override
    public boolean isFinished() {

        if (super.isFinished()) {
            // Oops, too close. We're done.
            return true;
        }

        Translation2d current = swerve.getPose().getTranslation();
        Translation2d target  = getTargetPose().getTranslation();
        return Math.abs(current.getDistance(target)) <= endWhileDrivingWhenThisCloseMetres;

    }

}
