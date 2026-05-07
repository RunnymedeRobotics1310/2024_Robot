package frc.robot.subsystems.swerve;

import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.config.GyroConfig;

public record SwerveDriveSubsystemConfig(
    boolean enabled,
    CoreSwerveConfig coreConfig,
    GyroConfig gyroConfig,
    SwerveTranslationConfig translationConfig,
    SwerveRotationConfig rotationConfig
) {}
