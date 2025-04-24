package frc.robot.subsystems.swerve;

import ca.team1310.swerve.core.config.CoreSwerveConfig;
import frc.robot.subsystems.vision.VisionConfig;

public record SwerveDriveSubsystemConfig(
    boolean enabled,
    CoreSwerveConfig coreConfig,
    VisionConfig visionConfig,
    SwerveTranslationConfig translationConfig,
    SwerveRotationConfig rotationConfig
) {}
