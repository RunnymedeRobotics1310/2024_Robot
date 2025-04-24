package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVisionSubsystem extends SubsystemBase {

    private static final long CAM_MODE_VISION = 0;
    private static final long PIPELINE_APRIL_TAG_DETECT = 0;

    NetworkTable lowRiderVision = NetworkTableInstance.getDefault().getTable("limelight-hugh");
    NetworkTable elevateVision = NetworkTableInstance.getDefault().getTable("limelight-jackman");

    // inputs/configs
    NetworkTableEntry lr_camMode = lowRiderVision.getEntry("camMode");
    NetworkTableEntry lr_pipeline = lowRiderVision.getEntry("pipeline");
    DoubleEntry lr_stream = lowRiderVision.getDoubleTopic("stream").getEntry(-1);

    // output
    DoubleArraySubscriber lr_MegaTag1 = lowRiderVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
    DoubleArraySubscriber lr_MegaTag2 = lowRiderVision.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);

    public enum CamStreamType {
        SIDE_BY_SIDE(0),
        LIMELIGHT(1),
        WEBCAM(2);

        private final int index;

        // Constructor for the enum, which assigns the index to each constant.
        private CamStreamType(int index) {
            this.index = index;
        }

        // Getter method to retrieve the index of the enum constant.
        public int getIndex() {
            return index;
        }
    }

    public enum TagType {
        RED_SOURCE_LEFT(1),
        RED_SOURCE_RIGHT(2),
        RED_PROCESSOR(3),
        RED_REEF_1(6),
        RED_REEF_2(7),
        RED_REEF_3(8),
        RED_REEF_4(9),
        RED_REEF_5(10),
        RED_REEF_6(11),
        BLUE_SOURCE_LEFT(12),
        BLUE_SOURCE_RIGHT(13),
        BLUE_PROCESSOR(16),
        BLUE_REEF_1(17),
        BLUE_REEF_2(18),
        BLUE_REEF_3(19),
        BLUE_REEF_4(20),
        BLUE_REEF_5(21),
        BLUE_REEF_6(22);

        private int tag;

        // Constructor for the enum, which assigns the index to each constant.
        private TagType(int tag) {
            this.tag = tag;
        }

        // Getter method to retrieve the index of the enum constant.
        public int getIndex() {
            return tag;
        }
    }

    private final LimelightBotPose limelightBotPose = new LimelightBotPose(null, 0);
    private int targetTagId = 0;

    public LimelightVisionSubsystem() {
        this.lr_pipeline.setNumber(PIPELINE_APRIL_TAG_DETECT);
        this.lr_camMode.setNumber(CAM_MODE_VISION);
    }

    @Override
    public void periodic() {
        TimestampedDoubleArray botPoseBlueMegaTag1 = lr_MegaTag1.getAtomic();
        limelightBotPose.update(botPoseBlueMegaTag1.value, botPoseBlueMegaTag1.timestamp);
    }

    /* Public API */

    public void setTargetTagId(TagType tag) {
        this.targetTagId = tag.getIndex();
    }

    public void clearTargetTagId() {
        this.targetTagId = 0;
    }

    public double getVisibleTargetTagId() {
        return limelightBotPose.getTagId(0);
    }

    public double distanceToTarget() {
        int index = 0;
        if (targetTagId > 0) {
            index = limelightBotPose.getTagIndex(targetTagId);
        }
        return limelightBotPose.getTagDistToRobot(index);
    }

    public double angleToTarget() {
        int index = 0;
        if (targetTagId > 0) {
            index = limelightBotPose.getTagIndex(targetTagId);
        }
        return limelightBotPose.getTagTxnc(index);
    }

    /**
     * Set the camera view to the specified stream.
     * @param stream the camera stream to set the view to
     */
    public void setCameraView(CamStreamType stream) {
        lr_stream.set(stream.getIndex());
    }

    public LimelightBotPose getBotPose() {
        return limelightBotPose;
    }

    @Override
    public String toString() {
        return "Hugh Vision Subsystem";
    }

}
