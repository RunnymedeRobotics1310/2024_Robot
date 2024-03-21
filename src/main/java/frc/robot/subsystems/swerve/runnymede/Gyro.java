package frc.robot.subsystems.swerve.runnymede;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SerialPort;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.telemetry.Telemetry;

class Gyro {
    private final AHRS gyro;
    private Rotation3d offset;

    Gyro() {
        gyro   = new AHRS(SerialPort.Port.kMXP);
        offset = gyro.getRotation3d();
    }

    void zeroGyro() {
        offset = gyro.getRotation3d();
    }

    Rotation3d getRotation3d() {
        return gyro.getRotation3d().minus(offset);
    }

    Rotation2d getRotation2d() {
        return gyro.getRotation2d().minus(offset.toRotation2d());
    }

    void updateTelemetry() {
        Telemetry.swerve.rawImuDegrees      = gyro.getRotation2d().getDegrees();
        Telemetry.swerve.adjustedImuDegrees = getRotation2d().getDegrees();
    }
}
