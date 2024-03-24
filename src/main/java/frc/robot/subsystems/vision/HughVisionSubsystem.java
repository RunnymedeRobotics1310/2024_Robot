package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.BotTarget;
import frc.robot.subsystems.RunnymedeSubsystemBase;
import frc.robot.telemetry.Telemetry;

/**
 * Handles the April Tag Limelight On Shooter Side
 */
public class HughVisionSubsystem extends RunnymedeSubsystemBase {

    private static final long          CAM_MODE_VISION                      = 0;
    private static final long          CAM_MODE_DRIVER                      = 1;

    // configure more pipelines here
    @SuppressWarnings("unused")
    private static final long          PIPELINE_APRIL_TAG_DETECT            = 0;
    private static final long          PIPELINE_RETROREFLECTIVE_NOTE_DETECT = 1;
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

    private static final int           BOTPOSE_INDEX_TX                     = 0;
    private static final int           BOTPOSE_INDEX_TY                     = 1;
    private static final int           BOTPOSE_INDEX_TZ                     = 2;
    private static final int           BOTPOSE_INDEX_R                      = 3;
    private static final int           BOTPOSE_INDEX_P                      = 4;
    private static final int           BOTPOSE_INDEX_Y                      = 5;
    private static final int           BOTPOSE_INDEX_LATENCY                = 6;
    private static final int           BOTPOSE_INDEX_TAGCOUNT               = 7;
    private static final int           BOTPOSE_INDEX_TAGSPAN                = 8;
    private static final int           BOTPOSE_INDEX_AVGDIST                = 9;
    private static final int           BOTPOSE_INDEX_AVGAREA                = 10;

    NetworkTableEntry                  targetpose_robotspace                = table.getEntry("targetpose_robotspace");

    NetworkTableEntry                  tid                                  = table.getEntry("tid");

    NetworkTableEntry                  priorityid                           = table.getEntry("priorityid");

    NetworkTableEntry                  json                                 = table.getEntry("json");

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

    public static class RawFiducial {
        public int id;
        public double txnc;
        public double tync;
        public double ta;
        public double distToCamera;
        public double distToRobot;
        public double ambiguity;


        public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }
    }

    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;
        public RawFiducial[] rawFiducials;

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
                            int tagCount, double tagSpan, double avgTagDist,
                            double avgTagArea, RawFiducial[] rawFiducials) {

            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
            this.rawFiducials = rawFiducials;
        }
    }


    public HughVisionSubsystem() {
        this.pipeline.setNumber(PIPELINE_APRIL_TAG_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
    }

    @Override
    public void periodic() {
        // read values periodically and post to smart dashboard periodically
        double[]           bp         = getBotPose();
        // AprilTagInfo[] visibleTags = getVisibleTagInfo();
        double             avgDist    = getTargetAvgDistance();
        int                numTargets = getNumActiveTargets();

        Telemetry.hugh.botTarget              = getBotTarget();
        Telemetry.hugh.priorityId             = getPriorityId();
        // Telemetry.hugh.targetFound = isCurrentTargetVisible();
        Telemetry.hugh.tid                    = tid.getDouble(-1.0);
        Telemetry.hugh.tx                     = tx.getDouble(-1.0);
        Telemetry.hugh.ty                     = ty.getDouble(-1.0);
        Telemetry.hugh.ta                     = ta.getDouble(-1.0);
        Telemetry.hugh.tl                     = tl.getDouble(-1.0);
        Telemetry.hugh.botpost                = bp;
        Telemetry.hugh.targetAvgDist          = avgDist;
        Telemetry.hugh.numTags                = getNumActiveTargets();
        Telemetry.hugh.distanceToTargetMetres = getDistanceToTargetMetres();
        Telemetry.hugh.isAlignedWithTarget    = isAlignedWithTarget();
        Telemetry.hugh.targetOffset           = getTargetOffset();
        Rotation2d r = getDynamicSpeakerShooterAngle(new Translation2d(0, 0));
        Telemetry.hugh.shooterAngle           = r == null ? Double.MIN_VALUE : r.getDegrees();
        // Telemetry.hugh.aprilTagInfo = aprilTagInfoArrayToString(visibleTags);
    }


    /**
     * Get the limelight coordinates for the target (i.e. with respect to the limelight origin, NOT
     * the robot!!)
     *
     * @return limelight target coordinates
     */
    private double[] getTarget() {
        double[] d = new double[2];
        d[0] = tx.getDouble(Double.MIN_VALUE);
        d[1] = ty.getDouble(Double.MIN_VALUE);
        return d;
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
     * Get the limelight Y angle measurement to the target.
     *
     * @return limelight Y target coordinates
     */
    private double getTargetY() {
        return ty.getDouble(Double.MIN_VALUE);
    }

    /**
     * Sets the priority Tag ID
     *
     * @return limelight Y target coordinates
     */
    private void setPriorityId(double tagId) {
        priorityid.setDouble(tagId);
    }

    /**
     * Gets the priority Tag ID
     *
     * @return priority tag id. -1 means no priority
     */
    private double getPriorityId() {
        return priorityid.getDouble(-1);
    }

    /**
     * Gets the average target distance
     *
     * @return Average distance to target in meters. If no value, returns Double.MAX_VALUE.
     */
    private double getTargetAvgDistance() {
        double[] botPose = getBotPose();
        if (botPose == null || botPose.length < 11 || botPose[BOTPOSE_INDEX_AVGDIST] == Double.MIN_VALUE) {
            return Double.MAX_VALUE;
        }

        return botPose[BOTPOSE_INDEX_AVGDIST];
    }

    /**
     * Performs a String based parsing of limelight's json blob in order to obtain9 information on
     * multiple targets when they are in view, since limelight only gives easy access to the
     * closest/largest one. Not using JSON parsers libs due to up to 2.5ms parsing time.
     *
     * @return An array of AprilTagInfo objects, each representing a visible target.
     */
    private AprilTagInfo[] getVisibleTagInfo() {
        String                  jsonStr = json.getString(null);
        ArrayList<AprilTagInfo> tags    = new ArrayList<AprilTagInfo>();

        if (jsonStr != null) {
            int index = 0;
            while (index != -1) {
                index = jsonStr.indexOf("\"fID\":", index);
                if (index == -1)
                    break; // No more fID found

                // Get Tag ID
                int    fIDStart    = index + 6;
                int    fIDEnd      = jsonStr.indexOf(",", fIDStart);
                String fID         = jsonStr.substring(fIDStart, fIDEnd).trim();

                // Get yTranslation (1st element of array)
                int    yTransIndex = jsonStr.indexOf("\"t6t_rs\":", fIDEnd);
                int    yTransStart = yTransIndex + 10;
                int    yTransEnd   = jsonStr.indexOf(",", yTransStart);
                String yTransStr   = jsonStr.substring(yTransStart, yTransEnd).trim();

                // Get xTranslation (3rd element of array)
                int    xTransIndex = jsonStr.indexOf(",", yTransEnd + 1);
                int    xTransStart = xTransIndex + 1;
                int    xTransEnd   = jsonStr.indexOf(",", xTransStart);
                String xTransStr   = jsonStr.substring(xTransStart, xTransEnd).trim();

                // Get xOffset
                int    txIndex     = jsonStr.indexOf("\"tx\":", fIDEnd);
                int    txStart     = txIndex + 5;
                int    txEnd       = jsonStr.indexOf(",", txStart);
                if (txEnd == -1) { // Check if tx is the last value before the object ends
                    txEnd = jsonStr.indexOf("}", txStart);
                }
                String       tx             = jsonStr.substring(txStart, txEnd).trim();

                int          tagId          = Integer.parseInt(fID);
                double       xOffset        = Double.parseDouble(tx);
                double       xTrans         = Double.parseDouble(xTransStr);
                double       yTrans         = Double.parseDouble(yTransStr);
                double       targetDistance = Math.hypot(xTrans, yTrans);

                AprilTagInfo ati            = new AprilTagInfo(tagId, xOffset, xTrans, yTrans, targetDistance);
                tags.add(ati);

                index = txEnd; // Move index to end of the current tx to find the next fID
            }
        }

        AprilTagInfo[] tagRet = new AprilTagInfo[tags.size()];
        return tags.toArray(tagRet);
    }

    private String aprilTagInfoArrayToString(AprilTagInfo[] tagArray) {
        if (tagArray == null || tagArray.length == 0) {
            return "[]";
        }

        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < tagArray.length; i++) {
            AprilTagInfo tag = tagArray[i];
            sb.append("[Tag:");
            sb.append(tag.tagId());
            sb.append(",xDeg:");
            sb.append(tag.xAngle());
            sb.append(",Dist:");
            sb.append(tag.targetDistance());
            sb.append("]");
            if (i < tagArray.length - 1) {
                sb.append(",");
            }
        }
        sb.append("]");
        return sb.toString();
    }

    private int getNumActiveTargets() {
        double[] botPose = getBotPose();
        if (botPose == null || botPose.length < 11 || botPose[BOTPOSE_INDEX_TAGCOUNT] == Double.MIN_VALUE) {
            return 0;
        }

        return (int) botPose[BOTPOSE_INDEX_TAGCOUNT];
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

    private static double extractBotPoseEntry(double[] inData, int position){
        if(inData.length < position+1)
        {
            return 0;
        }
        return inData[position];
    }

    private PoseEstimate getBotPoseEstimate() {
        var poseArray = botpose_wpiblue.getDoubleArray(new double[0]);
        var pose = toPose2D(poseArray);
        double latency = extractBotPoseEntry(poseArray,6);
        int tagCount = (int)extractBotPoseEntry(poseArray,7);
        double tagSpan = extractBotPoseEntry(poseArray,8);
        double tagDist = extractBotPoseEntry(poseArray,9);
        double tagArea = extractBotPoseEntry(poseArray,10);
        //getlastchange() in microseconds, ll latency in milliseconds
        var timestamp = (botpose_wpiblue.getLastChange() / 1000000.0) - (latency/1000.0);

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial*tagCount;

        if (poseArray.length != expectedTotalVals) {
            // Don't populate fiducials
        }
        else{
            for(int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int)poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(pose, timestamp,latency,tagCount,tagSpan,tagDist,tagArea,rawFiducials);
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
    public VisionPositionInfo getPositionInfo() {
        PoseEstimate poseEstimate = getBotPoseEstimate();

        // If pose is 0,0 or no tags in view, we don't actually have data - return null
        if ((poseEstimate.pose.getX() == 0 && poseEstimate.pose.getY() == 0)
                || poseEstimate.rawFiducials.length == 0) {
            return null;
        }

        // Get the "best" tag - assuming the first one is the best - TBD TODO
        RawFiducial rawFiducial = poseEstimate.rawFiducials[0];

        if (rawFiducial.ambiguity > Constants.VisionConstants.MAX_AMBIGUITY) {
            return null;
        }

        double stdDevRatio;

        // If the ambiguity is very low, use the data as is
        if (rawFiducial.ambiguity < Constants.VisionConstants.HIGH_QUALITY_AMBIGUITY) {
            stdDevRatio = .01;
        } else {
            stdDevRatio = Math.pow(rawFiducial.distToRobot,2) / 2;
        }

        Matrix<N3, N1> deviation = VecBuilder.fill(stdDevRatio, stdDevRatio, 5 * stdDevRatio);

        return new VisionPositionInfo(poseEstimate.pose, poseEstimate.timestampSeconds, deviation);
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

    /**
     * If any April tag in the actively set bot target is visible, return true.
     *
     * @return true if any April tag in the actively set bot target is visible
     * @since 2024-02-10
     */
    public boolean isCurrentTargetVisible() {
        AprilTagInfo[] visibleTags = getVisibleTagInfo();
        return Arrays.stream(visibleTags).anyMatch(tag -> activeAprilTagTargets.contains(tag.tagId()));
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

        // TODO: Use dynamic shooter location once available.  For now it's based on 185deg link
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