package frc.robot.utils;

public class SpeakerShooterLinearAngleCalc {
    // Splitting the dataset based on the boundary at 2.91m
    private static final double boundary = 2.91;

    // First segment (<= 2.91m)
    private static final double[] distancesBelowBoundary = {1.64, 1.74, 1.84, 1.94, 2.04, 2.14, 2.24, 2.34, 2.44, 2.54, 2.64, 2.74, 2.84, 2.91};
    private static final double[] anglesBelowBoundary = {30, 31.2, 32.8, 34.2, 35.6, 37, 38.4, 39.6, 40.8, 42, 43.2, 44.5, 46, 47.8};

    // Second segment (> 2.91m)
    private static final double[] distancesAboveBoundary = {2.91, 3.04, 3.14, 3.24, 3.34, 3.44, 3.54, 3.64, 3.74, 3.84};
    private static final double[] anglesAboveBoundary = {47.8, 48.2, 48.6, 49, 49.6, 50, 50, 50, 50.6, 51};

    public static double calculateAimAngle(double distance) {
        if (distance <= boundary) {
            return interpolate(distance, distancesBelowBoundary, anglesBelowBoundary);
        } else {
            return interpolate(distance, distancesAboveBoundary, anglesAboveBoundary);
        }
    }

    private static double interpolate(double distance, double[] segmentDistances, double[] segmentAngles) {
        // Check bounds first
        if (distance <= segmentDistances[0]) return segmentAngles[0];
        if (distance >= segmentDistances[segmentDistances.length - 1]) return segmentAngles[segmentAngles.length - 1];

        // Find the segment the distance falls into and interpolate
        for (int i = 0; i < segmentDistances.length - 1; i++) {
            if (distance >= segmentDistances[i] && distance <= segmentDistances[i + 1]) {
                // Perform linear interpolation
                double ratio = (distance - segmentDistances[i]) / (segmentDistances[i + 1] - segmentDistances[i]);
                return segmentAngles[i] + ratio * (segmentAngles[i + 1] - segmentAngles[i]);
            }
        }

        // Default to closest known angle if something goes wrong
        return segmentAngles[segmentAngles.length - 1];
    }

}
