package frc.robot.subsystems.vision;

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
import frc.robot.Constants.BotTarget;
import frc.robot.subsystems.RunnymedeSubsystemBase;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceHigh;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceLow;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceMedium;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceNone;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Telemetry;
import frc.robot.util.RectanglePoseArea;

import static frc.robot.Constants.LightingConstants.VISPOSE1;
import static frc.robot.Constants.LightingConstants.VISPOSE2;

/**
 * Handles the April Tag Limelight On Shooter Side
 */
public class HughVisionSubsystem extends RunnymedeSubsystemBase {

    private static final long          CAM_MODE_VISION                      = 0;
    @SuppressWarnings("unused")
    private static final long          CAM_MODE_DRIVER                      = 1;

    // configure more pipelines here
    private static final long          PIPELINE_APRIL_TAG_DETECT            = 0;
    @SuppressWarnings("unused")
    private static final long          PIPELINE_RETROREFLECTIVE_NOTE_DETECT = 1;
    @SuppressWarnings("unused")
    private static final long          PIPELINE_VISUAL                      = 2;

    private static final double        TARGET_ALIGNMENT_THRESHOLD           = 7.5;

    NetworkTable                       table                                = NetworkTableInstance.getDefault()
        .getTable("limelight-hugh");

    // inputs/configs
    NetworkTableEntry                  camMode                              = table.getEntry("camMode");
    NetworkTableEntry                  pipeline                             = table.getEntry("pipeline");

    // output
    NetworkTableEntry                  tv                                   = table.getEntry("tv");
    NetworkTableEntry                  tx                                   = table.getEntry("tx");
    NetworkTableEntry                  ty                                   = table.getEntry("ty");
    NetworkTableEntry                  ta                                   = table.getEntry("ta");

    NetworkTableEntry                  tl                                   = table.getEntry("tl");
    NetworkTableEntry                  cl                                   = table.getEntry("cl");

    NetworkTableEntry                  botpose_wpiblue                      = table.getEntry("botpose_wpiblue");
    NetworkTableEntry                  botpose_wpired                       = table.getEntry("botpose_wpibred");

    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_TX                     = 0;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_TY                     = 1;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_TZ                     = 2;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_R                      = 3;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_P                      = 4;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_Y                      = 5;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_LATENCY                = 6;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_TAGCOUNT               = 7;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_TAGSPAN                = 8;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_AVGDIST                = 9;
    @SuppressWarnings("unused")
    private static final int           BOTPOSE_INDEX_AVGAREA                = 10;

    NetworkTableEntry                  targetpose_robotspace                = table.getEntry("targetpose_robotspace");

    NetworkTableEntry                  tid                                  = table.getEntry("tid");

    NetworkTableEntry                  priorityId                           = table.getEntry("priorityid");

    private BotTarget                  botTarget                            = BotTarget.NONE;

    private static final List<Integer> TARGET_BLUE_SPEAKER                  = List.of(7, 8);
    private static final List<Integer> TARGET_BLUE_SOURCE                   = List.of(9, 10);
    private static final List<Integer> TARGET_BLUE_AMP                      = List.of(6);
    private static final List<Integer> TARGET_BLUE_STAGE                    = List.of(14, 15, 16);

    private static final List<Integer> TARGET_RED_SPEAKER                   = List.of(4, 3);
    private static final List<Integer> TARGET_RED_SOURCE                    = List.of(1, 2);
    private static final List<Integer> TARGET_RED_AMP                       = List.of(5);
    private static final List<Integer> TARGET_RED_STAGE                     = List.of(11, 12, 13);
    private static final List<Integer> TARGET_NONE                          = List.of();
    private static final List<Integer> TARGET_ALL                           = List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
        14, 15, 16);

    private List<Integer>              activeAprilTagTargets                = TARGET_ALL;

    private static final double        SPEAKER_TAG_DELTA                    = 0.565868;

    private SwerveSubsystem swerveSubsystem;
    private LightingSubsystem lightingSubsystem;


    private static final RectanglePoseArea FIELD_BOUNDARY = new RectanglePoseArea(new Translation2d(0, 0), new Translation2d(16.541, 8.211));
    private final NetworkTable poseTable = NetworkTableInstance.getDefault().getTable("LimelightPose");
    private final DoubleArrayPublisher limelightPub = poseTable.getDoubleArrayTopic("Robot").publish();

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

    public HughVisionSubsystem(SwerveSubsystem swerveSubsystem, LightingSubsystem lightingSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.lightingSubsystem = lightingSubsystem;
        this.pipeline.setNumber(PIPELINE_APRIL_TAG_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
    }

    @Override
    public void periodic() {
        // read values periodically and post to smart dashboard periodically
        PoseEstimate poseEstimate = getBotPoseEstimate();

        // Calc Position Info and Update Odometry, Lighting
        VisionPositionInfo visPosInfo = calcVisionPositionInfo(poseEstimate, swerveSubsystem.getPose());
        swerveSubsystem.updateOdometryWithVisionInfo(visPosInfo);
        updateLighting(visPosInfo.confidence());

        // Update Telemetry
        updateTelemetry(poseEstimate, visPosInfo);
    }

    private void updateLighting(PoseConfidence confidence) {
        switch (confidence) {
            case HIGH:
                lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceHigh.getInstance());
                lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceHigh.getInstance());
                break;
            case MEDIUM:
                lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceMedium.getInstance());
                lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceMedium.getInstance());
                break;
            case LOW:
                lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceLow.getInstance());
                lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceLow.getInstance());
                break;
            case NONE:
                lightingSubsystem.setPattern(VISPOSE1, VisionConfidenceNone.getInstance());
                lightingSubsystem.setPattern(VISPOSE2, VisionConfidenceNone.getInstance());
                break;
        }
    }

    private void updateTelemetry(PoseEstimate poseEstimate, VisionPositionInfo visPosInfo) {
        if (Telemetry.config.hugh()) {
            publishToField(poseEstimate.pose);
            Telemetry.hugh.poseUpdate = visPosInfo.confidence() != PoseConfidence.NONE;
            Telemetry.hugh.poseConfidence = visPosInfo.confidence();
            Telemetry.hugh.botTarget = getBotTarget();
            Telemetry.hugh.priorityId = getPriorityId();
            Telemetry.hugh.targetFound = isCurrentTargetVisible(poseEstimate);
            Telemetry.hugh.tid = tid.getDouble(-1.0);
            Telemetry.hugh.tx = tx.getDouble(-1.0);
            Telemetry.hugh.ty = ty.getDouble(-1.0);
            Telemetry.hugh.ta = ta.getDouble(-1.0);
            Telemetry.hugh.tl = tl.getDouble(-1.0);
            Telemetry.hugh.botpose = poseEstimate.pose.getTranslation().toVector().getData();
            Telemetry.hugh.targetAvgDist = poseEstimate.avgTagDist;
            Telemetry.hugh.numTags = poseEstimate.tagCount;
            Telemetry.hugh.distanceToTargetMetres = getDistanceToTargetMetres();
            Telemetry.hugh.isAlignedWithTarget = isAlignedWithTarget();
            Telemetry.hugh.targetOffset = getTargetOffset();
            Rotation2d r = getDynamicSpeakerShooterAngle(new Translation2d(0, 0));
            Telemetry.hugh.shooterAngle = r == null ? 1310 : r.getDegrees();
            Telemetry.hugh.aprilTagInfo = aprilTagInfoArrayToString(poseEstimate.rawFiducials);
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
     * Get the limelight X angle measurement to the target.
     *
     * @return limelight X target coordinates
     */
    private double getTargetX() {
        return tx.getDouble(Double.MIN_VALUE);
    }

    /**
     * Sets the priority Tag ID
     *
     * @return limelight Y target coordinates
     */
    private void setPriorityId(double tagId) {
        priorityId.setDouble(tagId);
    }

    /**
     * Gets the priority Tag ID
     *
     * @return priority tag id. -1 means no priority
     */
    private double getPriorityId() {
        return priorityId.getDouble(-1);
    }

    private double[] getBotPose() {
        double[] botpose = botpose_wpiblue.getDoubleArray(new double[] { Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE,
                Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE,
                Double.MIN_VALUE, Double.MIN_VALUE });
        if (botpose[0] == Double.MIN_VALUE) {
            return null;
        }

        return botpose;
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
     *
     * PUBLIC API FROM HERE DOWN
     *
     */

    /**
     * Get the position of the robot as computed by the Vision Subsystem. Includes latency data.
     *
     * If no valid position can be returned (due to bad or erratic data, blocked view, etc.),
     * returns null
     *
     * @return position info or null
     * @since 2024-02-10
     */
    private VisionPositionInfo calcVisionPositionInfo(PoseEstimate poseEstimate, Pose2d odometryPose) {
        Pose2d pose = poseEstimate.pose;
        double timestampSeconds = poseEstimate.timestampSeconds;
        double stdDevRatio = 1310;
        PoseConfidence poseConfidence = PoseConfidence.NONE;
        double compareDistance = poseEstimate.pose.getTranslation().getDistance(odometryPose.getTranslation());

        // If pose is 0,0 or no tags in view, we don't actually have data - return null
        if (poseEstimate.pose.getX() > 0
                && poseEstimate.pose.getY() > 0
                && poseEstimate.rawFiducials.length >= 1
                && FIELD_BOUNDARY.isPoseWithinArea(poseEstimate.pose)) {

            // Get the "best" tag - assuming the first one is the best - TBD TODO
            RawFiducial rawFiducial = poseEstimate.rawFiducials[0];

            if (rawFiducial.ambiguity < Constants.VisionConstants.MAX_AMBIGUITY) {
                // If the ambiguity is very low, use the data as is (or when disabled, to allow for bot repositioning
                if (rawFiducial.ambiguity < Constants.VisionConstants.HIGH_QUALITY_AMBIGUITY || DriverStation.isDisabled()) {
                    stdDevRatio = .01;
                    poseConfidence = PoseConfidence.HIGH;
                }
                else {
                    // We need to be careful with this data set.  If the location is too far off,
                    // don't use it.  Otherwise scale confidence by distance.
                    if (compareDistance < Constants.VisionConstants.MAX_VISPOSE_DELTA_DISTANCE) {
                        stdDevRatio = Math.pow(rawFiducial.distToRobot, 2) / 2;
                        poseConfidence = PoseConfidence.MEDIUM;
                    }
                }
            }
        }

        Matrix<N3, N1> deviation = VecBuilder.fill(stdDevRatio, stdDevRatio, 5 * stdDevRatio);
        return new VisionPositionInfo(poseEstimate.pose, poseEstimate.timestampSeconds, deviation, poseConfidence, compareDistance);
    }


    /**
     * Return the current BotTarget
     *
     * @return the current BotTarget
     */
    public BotTarget getBotTarget() {
        return botTarget;
    }

    /**
     * Sets the subsystem up to be ready to target a specific field element.
     *
     * @param botTarget the field element to target
     */
    public void setBotTarget(BotTarget botTarget) {

        this.botTarget = botTarget;

        switch (botTarget) {
        case BLUE_SPEAKER -> activeAprilTagTargets = TARGET_BLUE_SPEAKER;
        case BLUE_AMP -> activeAprilTagTargets = TARGET_BLUE_AMP;
        case BLUE_SOURCE -> activeAprilTagTargets = TARGET_BLUE_SOURCE;
        case BLUE_STAGE -> activeAprilTagTargets = TARGET_BLUE_STAGE;
        case RED_SPEAKER -> activeAprilTagTargets = TARGET_RED_SPEAKER;
        case RED_AMP -> activeAprilTagTargets = TARGET_RED_AMP;
        case RED_SOURCE -> activeAprilTagTargets = TARGET_RED_SOURCE;
        case RED_STAGE -> activeAprilTagTargets = TARGET_RED_STAGE;
        case NONE -> activeAprilTagTargets = TARGET_NONE;
        case ALL -> activeAprilTagTargets = TARGET_ALL;
        default -> throw new IllegalArgumentException(botTarget.toString());
        }

        if (botTarget == BotTarget.ALL || botTarget == BotTarget.NONE) {
            setPriorityId(-1);
        }
        else {
            setPriorityId(activeAprilTagTargets.get(0));
        }
    }

    private boolean isCurrentTargetVisible(PoseEstimate poseEstimate) {
        return Arrays.stream(poseEstimate.rawFiducials).anyMatch(rawFiducial -> activeAprilTagTargets.contains(rawFiducial.id));
    }

    /**
     * Check if the robot is aligned to the target, within a certain threshold. Threshold is defined
     * in TARGET_ALIGNMENT_THRESHOLD constant.
     *
     * @return true if the robot is aligned to the target, and a target tag is visible.
     */
    public boolean isAlignedWithTarget() {
        Rotation2d targetOffset = getTargetOffset();
        return targetOffset != null
            && targetOffset.getDegrees() > 180 - TARGET_ALIGNMENT_THRESHOLD
            && targetOffset.getDegrees() < 180 + TARGET_ALIGNMENT_THRESHOLD;
    }

    /**
     * Obtain the straight line distance to target, if any of the target's tags are in sight.
     *
     * @return distance to target in meters, or Double.MIN_VALUE if no targets are visible.
     */
    public double getDistanceToTargetMetres() {
        Translation2d robotPosition = getRobotTranslationToTarget();
        if (robotPosition == null) {
            return Double.MIN_VALUE;
        }
        return Math.hypot(robotPosition.getX(), robotPosition.getY());
    }

    /**
     * if the limelight does not have a lock on the tag, the function will return Double.MIN_VALUE
     * so you should not use this function to set the arm angle
     * 
     * @param shooterXY
     * @return Angle the aim needs to be at relative to the floor - which needs some math done
     * to handle link angle not being 180.
     */
    // todo: fixme: return rotation2d for consistency with other APIs and to eliminate unit
    // ambiguity
    public Rotation2d getDynamicSpeakerShooterAngle(Translation2d shooterXY) {
        double distanceToTargetMeters = getDistanceToTargetMetres();

        if (distanceToTargetMeters == Double.MIN_VALUE) {
            return null;
        }

        // TODO: Use dynamic shooter location once available. For now it's based on 185deg link
        double shooterHeight                            = 0.80;
        double shooterDistanceOffsetFromMiddleOfBot     = 0.345;

        double shooterDistanceToWall                    = distanceToTargetMeters - shooterDistanceOffsetFromMiddleOfBot;
        double heightDifferenceBetweenShooterAndSpeaker = BotTarget.BLUE_SPEAKER.getLocation().getZ() - shooterHeight;
        double oppOverAdj                               = heightDifferenceBetweenShooterAndSpeaker / shooterDistanceToWall;
        double preCalculatedShooterAngle                = Math.atan(oppOverAdj);
        double dynamicSpeakerShooterAngle               = 90 - Math.toDegrees(preCalculatedShooterAngle);
        return Rotation2d.fromDegrees(dynamicSpeakerShooterAngle);
    }


    /**
     * Obtains the relative heading to the target, if any of the target's tags are in sight.
     *
     * @return Rotation2d with the angle to target from center of bot (0,0). null if no targets are
     * visible.
     */
    public Rotation2d getTargetOffset() {
        int      currentTagId         = (int) tid.getInteger(-1);
        double[] targetPoseInBotSpace = targetpose_robotspace.getDoubleArray(new double[] { Double.MIN_VALUE, Double.MIN_VALUE,
                Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE });
        double   angleToTag           = getTargetX();
        double   tagX                 = targetPoseInBotSpace[2];
        double   tagY                 = targetPoseInBotSpace[0];

        if (!activeAprilTagTargets.contains(currentTagId) || angleToTag == Double.MIN_VALUE
            || targetPoseInBotSpace[0] == Double.MIN_VALUE) {
            return null;
        }

        // If we are targeting the blue speaker, and the current tag is the one on the left of
        // centre, we need to do some math to determine proper location towards center
        if ((botTarget == BotTarget.BLUE_SPEAKER && currentTagId == 8) ||
            (botTarget == BotTarget.RED_SPEAKER && currentTagId == 3)) {
            double radiansToTarget = Math.toRadians(angleToTag);
            double y_offset        = Math.cos(radiansToTarget) * SPEAKER_TAG_DELTA;
            double x_offset        = Math.sin(radiansToTarget) * SPEAKER_TAG_DELTA;

            double newTagX         = tagX + x_offset;
            double newTagY         = tagY + y_offset;

            angleToTag = Math.toDegrees(Math.atan2(newTagY, newTagX));
        }

        // Limelight is Clockwise Positive, but Drive/Odemetry is CCW positive, so we switch signs
        return Rotation2d.fromDegrees(-angleToTag);
    }

    /**
     * Obtains the x & y translation of the robot to the target, if any of the target's tags are in
     * sight.
     *
     * @return Translation2d with the x&y to target from center of bot (0,0). null if no targets are
     * visible.
     */
    public Translation2d getRobotTranslationToTarget() {
        int      currentTagId         = (int) tid.getInteger(-1);
        double[] targetPoseInBotSpace = targetpose_robotspace.getDoubleArray(new double[] { Double.MIN_VALUE, Double.MIN_VALUE,
                Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE });

        if (!activeAprilTagTargets.contains(currentTagId) || targetPoseInBotSpace[0] == Double.MIN_VALUE) {
            return null;
        }

        // Limelight is returning [0] = X = Left/Right, [1] = Y = Height, [2] = Z = Forward/Back.
        // We need to return X = Forward/Back, and Y = Left/Right
        // Because Hugh is on the back of the Bot, X needs to have its sign reversed.
        return new Translation2d(-targetPoseInBotSpace[2], targetPoseInBotSpace[0]);
    }

    @Override
    public String toString() {
        return "Hugh Vision Subsystem";
    }
}