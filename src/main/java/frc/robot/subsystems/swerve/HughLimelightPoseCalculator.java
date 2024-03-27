package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.vision.PoseConfidence;
import frc.robot.subsystems.vision.VisionPositionInfo;
import frc.robot.telemetry.Telemetry;
import frc.robot.util.RectanglePoseArea;

/**
 * Handles the April Tag Limelight
 */
public class HughLimelightPoseCalculator {

    private static final long              CAM_MODE_VISION                      = 0;
    @SuppressWarnings("unused")
    private static final long              CAM_MODE_DRIVER                      = 1;

    // configure more pipelines here
    private static final long              PIPELINE_APRIL_TAG_DETECT            = 0;
    @SuppressWarnings("unused")
    private static final long              PIPELINE_RETROREFLECTIVE_NOTE_DETECT = 1;
    @SuppressWarnings("unused")
    private static final long              PIPELINE_VISUAL                      = 2;

    NetworkTable                           table                                = NetworkTableInstance.getDefault()
        .getTable("limelight-hugh");

    // inputs/configs
    private final NetworkTableEntry        camMode                              = table.getEntry("camMode");
    private final NetworkTableEntry        pipeline                             = table.getEntry("pipeline");

    // output
    private final NetworkTableEntry        tv                                   = table.getEntry("tv");
    private final NetworkTableEntry        tx                                   = table.getEntry("tx");
    private final NetworkTableEntry        ty                                   = table.getEntry("ty");
    private final NetworkTableEntry        ta                                   = table.getEntry("ta");

    private final NetworkTableEntry        tl                                   = table.getEntry("tl");
    private final NetworkTableEntry        cl                                   = table.getEntry("cl");

    private final NetworkTableEntry        botpose_wpiblue                      = table.getEntry("botpose_wpiblue");
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_TX                     = 0;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_TY                     = 1;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_TZ                     = 2;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_R                      = 3;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_P                      = 4;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_Y                      = 5;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_LATENCY                = 6;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_TAGCOUNT               = 7;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_TAGSPAN                = 8;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_AVGDIST                = 9;
    @SuppressWarnings("unused")
    private static final int               BOTPOSE_INDEX_AVGAREA                = 10;
    private final NetworkTableEntry        tid                                  = table.getEntry("tid");

    private final NetworkTableEntry        priorityId                           = table.getEntry("priorityid");

    private List<Integer>                  activeAprilTagTargets                = List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
        13, 14, 15, 16);
    private static final RectanglePoseArea FIELD_BOUNDARY                       = new RectanglePoseArea(new Translation2d(0, 0),
        new Translation2d(16.541, 8.211));
    private final NetworkTable             poseTable                            = NetworkTableInstance.getDefault()
        .getTable("LimelightPose");
    private final DoubleArrayPublisher     limelightPub                         = poseTable.getDoubleArrayTopic("Robot")
        .publish();

    private static class RawFiducial {
        public int    id;
        public double txnc;
        public double tync;
        public double ta;
        public double distToCamera;
        public double distToRobot;
        public double ambiguity;

        public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot,
            double ambiguity) {
            this.id           = id;
            this.txnc         = txnc;
            this.tync         = tync;
            this.ta           = ta;
            this.distToCamera = distToCamera;
            this.distToRobot  = distToRobot;
            this.ambiguity    = ambiguity;
        }
    }

    private static class PoseEstimate {
        public Pose2d        pose;
        public double        timestampSeconds;
        public double        latency;
        public int           tagCount;
        public double        tagSpan;
        public double        avgTagDist;
        public double        avgTagArea;
        public RawFiducial[] rawFiducials;

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
            int tagCount, double tagSpan, double avgTagDist,
            double avgTagArea, RawFiducial[] rawFiducials) {

            this.pose             = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency          = latency;
            this.tagCount         = tagCount;
            this.tagSpan          = tagSpan;
            this.avgTagDist       = avgTagDist;
            this.avgTagArea       = avgTagArea;
            this.rawFiducials     = rawFiducials;
        }
    }

    public HughLimelightPoseCalculator() {
        this.pipeline.setNumber(PIPELINE_APRIL_TAG_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
    }

    /**
     * Compute the position information using limelight data and the current odometry pose.
     *
     * @return VisionPositionInfo. Never null, but if the confidence returned is null, this data
     * should not be used.
     */
    public VisionPositionInfo getVisionPositionInfo(Pose2d currentOdometryPose) {
        PoseEstimate       poseEstimate = getBotPoseEstimate();
        VisionPositionInfo visPoseInfo  = calcVisionPositionInfo(poseEstimate, currentOdometryPose);
        updateTelemetry(poseEstimate, visPoseInfo);
        return visPoseInfo;
    }

    private void updateTelemetry(PoseEstimate poseEstimate, VisionPositionInfo visPosInfo) {
        if (Telemetry.config.hugh()) {
            publishToField(poseEstimate.pose);
            Telemetry.hugh.poseUpdate     = visPosInfo.confidence() != PoseConfidence.NONE;
            Telemetry.hugh.poseConfidence = visPosInfo.confidence();
            Telemetry.hugh.priorityId     = getPriorityId();
            Telemetry.hugh.targetFound    = isCurrentTargetVisible(poseEstimate);
            Telemetry.hugh.tid            = tid.getDouble(-1.0);
            Telemetry.hugh.tx             = tx.getDouble(-1.0);
            Telemetry.hugh.ty             = ty.getDouble(-1.0);
            Telemetry.hugh.ta             = ta.getDouble(-1.0);
            Telemetry.hugh.tl             = tl.getDouble(-1.0);
            Telemetry.hugh.botpose        = poseEstimate.pose.getTranslation().toVector().getData();
            Telemetry.hugh.targetAvgDist  = poseEstimate.avgTagDist;
            Telemetry.hugh.numTags        = poseEstimate.tagCount;
            Telemetry.hugh.aprilTagInfo   = aprilTagInfoArrayToString(poseEstimate.rawFiducials);
            Telemetry.hugh.poseSwerveDiff = visPosInfo.odometryDistDelta();
        }
    }

    private String aprilTagInfoArrayToString(RawFiducial[] rawFiducials) {
        if (rawFiducials == null) {
            return "null";
        }
        StringBuilder sb = new StringBuilder();
        for (RawFiducial rawFiducial : rawFiducials) {
            sb.append("[id:").append(rawFiducial.id)
                .append(",distToRobot:").append(rawFiducial.distToRobot)
                .append(",ambiguity:").append(rawFiducial.ambiguity)
                .append(",txnc:").append(rawFiducial.txnc)
                .append(",tync:").append(rawFiducial.tync)
                .append(",ta:").append(rawFiducial.ta)
                .append("]");
        }
        return sb.toString();
    }

    /**
     * Gets the priority Tag ID
     *
     * @return priority tag id. -1 means no priority
     */
    private double getPriorityId() {
        return priorityId.getDouble(-1);
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        // Add 180deg to rotation because Hugh is on rear of bot
        Rotation2d    r2d    = Rotation2d.fromDegrees((inData[5] + 180) % 360);
        return new Pose2d(tran2d, r2d);
    }

    private static double extractBotPoseEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0;
        }
        return inData[position];
    }

    private PoseEstimate getBotPoseEstimate() {
        var           poseArray         = botpose_wpiblue.getDoubleArray(new double[0]);
        var           pose              = toPose2D(poseArray);
        double        latency           = extractBotPoseEntry(poseArray, 6);
        int           tagCount          = (int) extractBotPoseEntry(poseArray, 7);
        double        tagSpan           = extractBotPoseEntry(poseArray, 8);
        double        tagDist           = extractBotPoseEntry(poseArray, 9);
        double        tagArea           = extractBotPoseEntry(poseArray, 10);
        // getlastchange() in microseconds, ll latency in milliseconds
        var           timestampSeconds  = (botpose_wpiblue.getLastChange() / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials      = new RawFiducial[tagCount];
        int           valsPerFiducial   = 7;
        int           expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length != expectedTotalVals) {
            // Don't populate fiducials
        }
        else {
            for (int i = 0; i < tagCount; i++) {
                int    baseIndex    = 11 + (i * valsPerFiducial);
                int    id           = (int) poseArray[baseIndex];
                double txnc         = poseArray[baseIndex + 1];
                double tync         = poseArray[baseIndex + 2];
                double ta           = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot  = poseArray[baseIndex + 5];
                double ambiguity    = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(pose, timestampSeconds, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials);
    }

    private void publishToField(Pose2d llPose) {
        // If you have a Field2D you can easily push it that way here.
        limelightPub.set(new double[] {
                llPose.getX(),
                llPose.getY(),
                llPose.getRotation().getDegrees()
        });
    }

    /**
     * Get the position of the robot as computed by the Vision Subsystem. Includes latency data.
     * <p>
     * If no valid position can be returned (due to bad or erratic data, blocked view, etc.),
     * returns null
     *
     * @return position info or null
     * @since 2024-02-10
     */
    private VisionPositionInfo calcVisionPositionInfo(PoseEstimate poseEstimate, Pose2d odometryPose) {
        double         stdDevRatio     = 1310;
        PoseConfidence poseConfidence  = PoseConfidence.NONE;
        double         compareDistance = poseEstimate.pose.getTranslation().getDistance(odometryPose.getTranslation());

        // If pose is 0,0 or no tags in view, we don't actually have data - return null
        if (poseEstimate.pose.getX() > 0
            && poseEstimate.pose.getY() > 0
            && poseEstimate.rawFiducials.length >= 1
            && FIELD_BOUNDARY.isPoseWithinArea(poseEstimate.pose)) {

            // Get the "best" tag - assuming the first one is the best - TBD TODO
            RawFiducial rawFiducial = poseEstimate.rawFiducials[0];

            if (rawFiducial.ambiguity < Constants.VisionConstants.MAX_AMBIGUITY) {
                // If the ambiguity is very low, use the data as is (or when disabled, to allow for
                // bot repositioning
                if (rawFiducial.ambiguity < Constants.VisionConstants.HIGH_QUALITY_AMBIGUITY || DriverStation.isDisabled()) {
                    stdDevRatio    = .01;
                    poseConfidence = PoseConfidence.HIGH;
                }
                else {
                    // We need to be careful with this data set. If the location is too far off,
                    // don't use it. Otherwise scale confidence by distance.
                    if (compareDistance < Constants.VisionConstants.MAX_VISPOSE_DELTA_DISTANCE) {
                        stdDevRatio    = Math.pow(rawFiducial.distToRobot, 2) / 2;
                        poseConfidence = PoseConfidence.MEDIUM;
                    }
                }
            }
        }

        Matrix<N3, N1> deviation = VecBuilder.fill(stdDevRatio, stdDevRatio, 5 * stdDevRatio);
        return new VisionPositionInfo(poseEstimate.pose, poseEstimate.timestampSeconds, deviation, poseConfidence,
            compareDistance);
    }

    private boolean isCurrentTargetVisible(PoseEstimate poseEstimate) {
        return Arrays.stream(poseEstimate.rawFiducials).anyMatch(rawFiducial -> activeAprilTagTargets.contains(rawFiducial.id));
    }
}