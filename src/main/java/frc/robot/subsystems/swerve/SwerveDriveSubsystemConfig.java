package frc.robot.subsystems.swerve;

import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.vision.VisionConfig;

public record SwerveDriveSubsystemConfig(
    boolean enabled,
    CoreSwerveConfig coreConfig,
    VisionConfig visionConfig,
    SwerveTranslationConfig translationConfig,
    SwerveRotationConfig rotationConfig
) {}
