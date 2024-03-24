package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionPositionInfo(Pose2d pose, double latencySeconds, Matrix<N3, N1> deviation) {
    public String toString() {
        return String.format("(%.2f,%.2f)m %.0fd %.0fms ",
            pose.getTranslation().getX(),
            pose.getTranslation().getY(),
            pose.getRotation().getDegrees(),
            latencySeconds);
    }
}

