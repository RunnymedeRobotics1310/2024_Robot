package frc.robot.subsystems.swerve.runnymede;


import static edu.wpi.first.math.util.Units.metersToInches;
import static frc.robot.Constants.Swerve.Chassis.MAX_MODULE_SPEED_MPS;
import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATIONAL_VELOCITY_PER_SEC;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
import static frc.robot.Constants.Swerve.Module.BACK_LEFT;
import static frc.robot.Constants.Swerve.Module.BACK_RIGHT;
import static frc.robot.Constants.Swerve.Module.FRONT_LEFT;
import static frc.robot.Constants.Swerve.Module.FRONT_RIGHT;
import static frc.robot.Constants.Swerve.Motor.ANGLE;
import static frc.robot.Constants.Swerve.Motor.DRIVE;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.telemetry.Telemetry1310;

import java.util.Arrays;

/**
 * Represents a swerve drive style drivetrain.
 */
public class RunnymedeSwerveSubsystem extends SwerveSubsystem {
    private final SwerveModule[]          modules;
    private final SwerveDriveKinematics   kinematics;
    private final AHRS                    gyro;
    private final SimulatedIMU            simulatedIMU;
    private Rotation3d                    gyroOffset;
    public Field2d                        field;

    public final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public RunnymedeSwerveSubsystem(HughVisionSubsystem visionSubsystem, LightingSubsystem lightingSubsystem) {
        super(visionSubsystem, lightingSubsystem);

        modules      = new SwerveModule[4];
        modules[0]   = new SwerveModule(FRONT_LEFT, DRIVE, ANGLE);
        modules[1]   = new SwerveModule(FRONT_RIGHT, DRIVE, ANGLE);
        modules[2]   = new SwerveModule(BACK_LEFT, DRIVE, ANGLE);
        modules[3]   = new SwerveModule(BACK_RIGHT, DRIVE, ANGLE);

        kinematics   = new SwerveDriveKinematics(
            Arrays.stream(modules).map(SwerveModule::getLocation).toArray(Translation2d[]::new));

        gyro         = new AHRS(SerialPort.Port.kMXP);
        gyroOffset   = gyro.getRotation3d();
        simulatedIMU = new SimulatedIMU();


        field        = new Field2d();
        SmartDashboard.putData(field);
        Telemetry1310.swerveCore.maxSpeed           = Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
        Telemetry1310.swerveCore.maxAngularVelocity = Constants.Swerve.Chassis.MAX_ROTATIONAL_VELOCITY_PER_SEC.getDegrees();
        Telemetry1310.swerveCore.moduleCount        = modules.length;
        Telemetry1310.swerveCore.sizeFrontBack      = metersToInches(Constants.Swerve.Chassis.WHEEL_BASE_METRES);
        Telemetry1310.swerveCore.sizeLeftRight      = metersToInches(Constants.Swerve.Chassis.TRACK_WIDTH_METRES);
        Telemetry1310.swerveCore.wheelLocations     = new double[Telemetry1310.swerveCore.moduleCount * 2];
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            Telemetry1310.swerveCore.wheelLocations[i * 2]     = metersToInches(module.getLocation().getX());
            Telemetry1310.swerveCore.wheelLocations[i * 2 + 1] = metersToInches(module.getLocation().getY());
        }
        Telemetry1310.swerveCore.measuredStates = new double[Telemetry1310.swerveCore.moduleCount * 2];
        Telemetry1310.swerveCore.desiredStates  = new double[Telemetry1310.swerveCore.moduleCount * 2];


        this.swerveDrivePoseEstimator           = new SwerveDrivePoseEstimator(
            this.kinematics,
            gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new),
            new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
    }

    @Override
    public void setModuleStateForTestMode(Constants.Swerve.Module module, SwerveModuleState desiredState) {
        SwerveModule swerveModule = null;
        for (SwerveModule m : modules) {
            if (m.getName().equals(module.name)) {
                swerveModule = m;
            }
        }
        if (swerveModule == null) {
            System.out.println("Invalid module name: " + module.name);
            return;
        }

        // set the state
        swerveModule.setDesiredState(desiredState);
    }

    @Override
    public void updateTelemetry() {
        ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getStates());
        Telemetry1310.swerveCore.measuredChassisSpeeds[1] = measuredChassisSpeeds.vyMetersPerSecond;
        Telemetry1310.swerveCore.measuredChassisSpeeds[0] = measuredChassisSpeeds.vxMetersPerSecond;
        Telemetry1310.swerveCore.measuredChassisSpeeds[2] = Math.toDegrees(measuredChassisSpeeds.omegaRadiansPerSecond);
        Telemetry1310.swerveCore.robotRotation            = getPose().getRotation().getDegrees();
        Telemetry1310.swervePlus.rawImuDegrees            = gyro.getRotation3d().toRotation2d().getDegrees();
        Telemetry1310.swervePlus.adjustedImuDegrees       = gyro.getRotation3d().minus(gyroOffset).toRotation2d().getDegrees();

        for (int i = 0; i < modules.length; i++) {
            SwerveModule      module      = modules[i];
            SwerveModuleState moduleState = module.getState();
            Telemetry1310.swerveCore.measuredStates[i * 2]       = moduleState.angle.getDegrees();
            Telemetry1310.swerveCore.measuredStates[(i * 2) + 1] = moduleState.speedMetersPerSecond;
            module.updateTelemetry();
        }
    }

    @Override
    protected void driveRawRobotOriented(ChassisSpeeds velocity) {

        // calculate desired states
        ChassisSpeeds       discretized        = ChassisSpeeds.discretize(velocity, Robot.kDefaultPeriod);
        Translation2d       centerOfRotation   = new Translation2d();
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(discretized, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, velocity,
            MAX_MODULE_SPEED_MPS, MAX_TRANSLATION_SPEED_MPS, MAX_ROTATIONAL_VELOCITY_PER_SEC.getRadians());

        Telemetry1310.swerveCore.desiredChassisSpeeds[1] = velocity.vyMetersPerSecond;
        Telemetry1310.swerveCore.desiredChassisSpeeds[0] = velocity.vxMetersPerSecond;
        Telemetry1310.swerveCore.desiredChassisSpeeds[2] = Math.toDegrees(velocity.omegaRadiansPerSecond);

        // set states
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i]);
            Telemetry1310.swerveCore.desiredStates[i * 2]       = swerveModuleStates[i].angle.getDegrees();
            Telemetry1310.swerveCore.desiredStates[(i * 2) + 1] = swerveModuleStates[i].speedMetersPerSecond;

        }
    }

    @Override
    public void updateOdometryWithStates() {
        swerveDrivePoseEstimator.update(
            gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new));

        Pose2d robotPose = swerveDrivePoseEstimator.getEstimatedPosition();

        field.setRobotPose(robotPose);

        if (RobotBase.isSimulation()) {
            simulatedIMU.updateOdometry(kinematics, getStates(), getModulePoses(robotPose), field);
        }
    }

    private SwerveModuleState[] getStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    @Override
    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    private Pose2d[] getModulePoses(Pose2d robotPose) {
        return Arrays.stream(modules).map(m -> {
            Transform2d tx = new Transform2d(m.getLocation(), m.getState().angle);
            return robotPose.plus(tx);
        }).toArray(Pose2d[]::new);
    }

    @Override
    protected void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        this.swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
    }

    @Override
    public void zeroGyro() {
        gyroOffset = gyro.getRotation3d();
    }

    @Override
    public void lock() {
        // TODO: ADD SAFETY CODE

        // set speed to 0 and angle wheels to center
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(new SwerveModuleState(0.0, modules[i].getPosition().angle));
            Telemetry1310.swerveCore.desiredStates[i * 2]       = modules[i].getPosition().angle.getDegrees();
            Telemetry1310.swerveCore.desiredStates[(i * 2) + 1] = 0;
        }

        // tell kinematics that we aren't moving
        kinematics.toSwerveModuleStates(new ChassisSpeeds());

    }

    @Override
    public void resetOdometry(Pose2d pose) {
        this.swerveDrivePoseEstimator.resetPosition(gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new), pose);
    }

    @Override
    public String toString() {
        return "Runnymede " + super.toString();
    }
}