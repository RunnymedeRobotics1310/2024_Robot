package frc.robot.subsystems.swerve.runnymede;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.telemetry.Telemetry;

class SwerveModule {

    private final int                   INTERNAL_ENCODER_UPDATE_FREQ = 10;
    private int                         internalEncoderUpdateCount   = 0;
    private final String                name;
    private final Translation2d         location;
    private final DriveMotor            driveMotor;
    private final AngleMotor            angleMotor;
    private final CanCoder              encoder;
    private final SimulatedSwerveModule sim;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning
     * encoder.
     */
    SwerveModule(Constants.Swerve.Module cfg, Constants.Swerve.Motor driveCfg, Constants.Swerve.Motor angleCfg) {
        this.name     = cfg.name;
        this.location = cfg.locationMetres;
        driveMotor    = new DriveMotor(cfg.driveCANID, driveCfg, cfg.wheelRadiusMetres);
        angleMotor    = new AngleMotor(cfg.angleCANID, angleCfg);
        encoder       = new CanCoder(cfg.encoderCANID, cfg.encoderAbsoluteOffsetDegrees, false);

        sim           = new SimulatedSwerveModule();

        updateInternalEncoder();
    }

    String getName() {
        return name;
    }

    Translation2d getLocation() {
        return location;
    }

    SwerveModulePosition getPosition() {
        if (RobotBase.isSimulation()) {
            return sim.getPosition();
        }
        else {
            return new SwerveModulePosition(driveMotor.getDistanceMetres(), angleMotor.getPosition());
        }
    }

    SwerveModuleState getState() {
        if (RobotBase.isSimulation()) {
            return sim.getState();
        }
        else {
            double     velocity = driveMotor.getVelocityMetresPerSecond();
            Rotation2d azimuth  = angleMotor.getPosition();
            return new SwerveModuleState(velocity, azimuth);
        }
    }

    void setDesiredState(SwerveModuleState desiredState) {
        if (RobotBase.isSimulation()) {
            sim.setDesiredState(desiredState);
        }
        else {
            Rotation2d currentHeading = angleMotor.getPosition();

            // Optimize the reference state to avoid spinning further than 90 degrees
            desiredState = SwerveModuleState.optimize(desiredState, currentHeading);

            /*
             * If the angle error is close to 0 degrees, we are aligned properly, so we can apply
             * full power to drive wheels. If the angle error is close to 90 degrees, driving in
             * any direction does not help. Used cosine function on the error to scale the
             * desired speed. If cosine is < 0 then scale to zero so that we don't invert the
             * drive for no reason.
             */
            Rotation2d steerError   = desiredState.angle.minus(currentHeading);
            double     cosineScalar = steerError.getCos();
            desiredState.speedMetersPerSecond *= (cosineScalar < 0 ? 0 : cosineScalar);
            driveMotor.setReferenceMetresPerSecond(desiredState.speedMetersPerSecond, 0);

            angleMotor.setReferenceDegrees(desiredState.angle.getDegrees(), 0);
        }

        updateInternalEncoder();

        Telemetry.swerve.getModule(name).speedMetersPerSecond = desiredState.speedMetersPerSecond;
        Telemetry.swerve.getModule(name).angleDegrees         = desiredState.angle.getDegrees();
    }

    void updateInternalEncoder() {

        if (internalEncoderUpdateCount++ >= INTERNAL_ENCODER_UPDATE_FREQ) {
            internalEncoderUpdateCount = 0;
            double angle = encoder.getAbsolutePositionInDegrees();
            if (encoder.readingError) {
                System.out.println("Absolute encoder " + encoder.getDeviceId() + " could not be read.");
            }
            else {
                angleMotor.setInternalEncoderPositionDegrees(angle);
            }
        }
    }

    void updateTelemetry() {
        Telemetry.swerve.getModule(name).absoluteEncoderPositionDegrees = encoder.getAbsolutePositionInDegrees();
        Telemetry.swerve.getModule(name).angleMotorPosition             = angleMotor.getPosition();
        Telemetry.swerve.getModule(name).driveMotorPosition             = driveMotor.getDistanceMetres();
    }
}