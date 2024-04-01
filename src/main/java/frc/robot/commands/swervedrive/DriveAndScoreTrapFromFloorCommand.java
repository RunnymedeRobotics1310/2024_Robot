package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.UsefulPoses.*;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class DriveAndScoreTrapFromFloorCommand extends BaseDriveCommand {
    private final ArmSubsystem      arm;
    private final LightingSubsystem lighting;

    private Pose2d                  target = null;

    public DriveAndScoreTrapFromFloorCommand(SwerveSubsystem swerve, ArmSubsystem arm, LightingSubsystem lighting) {
        super(swerve);
        this.arm      = arm;
        this.lighting = lighting;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.target = getTargetPose(swerve.getPose(), getRunnymedeAlliance());
    }

    @Override
    public void execute() {
        super.execute();
        /*
         * todo: quentin: implement. Also see Constants.UsefulPoses.FACING_CHAIN_RED_LEFT etc. for
         * positions - you need to math it up.
         */

        // maybe use a state machine?
        // drive to pose
        // cool lights
        // set arm to position
        // shoot

    }

    @Override
    public boolean isFinished() {
        return !arm.isNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerve.stop();
        arm.stop();
        // remove lights
    }

    private static Pose2d getTargetPose(Pose2d pose, DriverStation.Alliance alliance) {

        if (alliance == DriverStation.Alliance.Blue) {
            if (pose.getX() > Constants.FieldConstants.WING_LENGTH_METRES) {
                // center stage
                return SCORE_TRAP_FWD_FROM_FLOOR_RED_CENTER;
            }
            else {
                if (pose.getY() > Constants.FieldConstants.FIELD_EXTENT_METRES_Y / 2) {
                    // stage left
                    return SCORE_TRAP_FWD_FROM_FLOOR_RED_LEFT;
                }
                else {
                    // stage right
                    return SCORE_TRAP_FWD_FROM_FLOOR_RED_RIGHT;
                }
            }
        }
        else {
            if (pose.getX() < Constants.FieldConstants.FIELD_EXTENT_METRES_X - Constants.FieldConstants.WING_LENGTH_METRES) {
                // center stage
                return SCORE_TRAP_FWD_FROM_FLOOR_BLUE_CENTER;
            }
            else {
                if (pose.getY() < Constants.FieldConstants.FIELD_EXTENT_METRES_Y / 2) {
                    // stage left
                    return SCORE_TRAP_FWD_FROM_FLOOR_BLUE_LEFT;
                }
                else {
                    // stage right
                    return SCORE_TRAP_FWD_FROM_FLOOR_BLUE_RIGHT;
                }
            }
        }
    }

}
