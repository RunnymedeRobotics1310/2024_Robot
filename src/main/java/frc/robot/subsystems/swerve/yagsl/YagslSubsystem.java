package frc.robot.subsystems.swerve.yagsl;

import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Swerve;
import frc.robot.telemetry.Telemetry;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class YagslSubsystem extends SwerveSubsystem {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;


    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param configDirectory Directory of swerve drive config files.
     */
    public YagslSubsystem(LightingSubsystem lighting, File configDirectory) {
        super(lighting);
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        Telemetry.swerve.implementation = Swerve.Implementation.YAGSL;
        SwerveDriveTelemetry.verbosity  = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(configDirectory).createSwerveDrive(MAX_TRANSLATION_SPEED_MPS);
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }
        // Runnymede does its own heading correction in the commands.
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a
                                 // starting
            // pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
                                      // ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                                             // in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the
                                       // options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.kinematics.toChassisSpeeds(swerveDrive.getStates());
    }

    @Override
    public void setModuleStateForTestMode(Constants.Swerve.Module module, SwerveModuleState desiredState) {
        SwerveModule swerveModule = swerveDrive.getModuleMap().get(module.name);
        if (swerveModule == null) {
            log("Invalid module name: " + module.name);
            return;
        }

        // save cosine compensator setting
        boolean coco = swerveModule.configuration.useCosineCompensator;
        swerveModule.configuration.useCosineCompensator = false;

        // set the state
        swerveModule.setDesiredState(desiredState, true, true);

        // restore the cosine compensator setting
        swerveModule.configuration.useCosineCompensator = coco;
    }

    @Override
    protected void driveRawRobotOriented(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity, false, new Translation2d());
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    public void updateTelemetry() {
        // noop - done internally inside SwerveDrive
    }

    @Override
    protected void updateOdometryWithStates() {
        // noop - done internally inside SwerveDrive
    }

    @Override
    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        swerveDrive.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
    }

    @Override
    public Rotation3d getGyroRotation3d() {
        return swerveDrive.getGyroRotation3d();
    }

    @Override
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    @Override
    public void lock() {
        swerveDrive.lockPose();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public String toString() {
        return "YAGSL " + super.toString();
    }
}
