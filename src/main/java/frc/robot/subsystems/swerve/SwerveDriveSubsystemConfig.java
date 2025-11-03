package frc.robot.subsystems.swerve;

import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.config.GyroConfig;
import frc.robot.subsystems.vision.VisionConfig;

public record SwerveDriveSubsystemConfig(
    boolean enabled,
    CoreSwerveConfig coreConfig,
    GyroConfig gyroConfig,
    VisionConfig visionConfig,
    SwerveTranslationConfig translationConfig,
    SwerveRotationConfig rotationConfig
) {}
