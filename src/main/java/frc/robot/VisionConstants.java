package frc.robot;

public final class VisionConstants {

    /** Time to switch pipelines and acquire a new vision target */
    public static final double VISION_SWITCH_TIME_SEC = .25;

    public enum VisionTarget {
        APRILTAGS,
        NOTES,
        NONE;

        @Override
        public String toString() {
            return "VisionTarget: " + name();
        }

    }

}
