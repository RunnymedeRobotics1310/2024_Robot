package frc.robot.subsystems.swerve.runnymede;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.Optional;

class SimulatedIMU {
    private final Timer timer = new Timer();
    private double      lastTime;
    private double      angle;

    SimulatedIMU() {
        this.timer.start();
        this.lastTime = this.timer.get();
    }

    Rotation2d getYaw() {
        return new Rotation2d(this.angle);
    }

    Rotation2d getPitch() {
        return new Rotation2d();
    }

    Rotation2d getRoll() {
        return new Rotation2d();
    }

    Rotation3d getGyroRotation3d() {
        return new Rotation3d(0.0, 0.0, this.angle);
    }

    Optional<Translation3d> getAccel() {
        return Optional.empty();
    }

    void updateOdometry(SwerveDriveKinematics kinematics, SwerveModuleState[] states, Pose2d[] modulePoses,
        Field2d field) {
        this.angle    += kinematics.toChassisSpeeds(states).omegaRadiansPerSecond * (this.timer.get() - this.lastTime);
        this.lastTime  = this.timer.get();
        field.getObject("XModules").setPoses(modulePoses);
    }

    void setAngle(double angle) {
        this.angle = angle;
    }
}
