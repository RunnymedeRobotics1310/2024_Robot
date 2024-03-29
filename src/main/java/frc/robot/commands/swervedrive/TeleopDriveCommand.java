package frc.robot.commands.swervedrive;

import static frc.robot.Constants.LightingConstants.SIGNAL;
import static frc.robot.Constants.ShooterConstants.SPEAKER_SHOT_RANGE_METRES;
import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;
import static frc.robot.commands.operator.OperatorInput.Axis.X;
import static frc.robot.commands.operator.OperatorInput.Axis.Y;
import static frc.robot.commands.operator.OperatorInput.Stick.LEFT;
import static frc.robot.commands.operator.OperatorInput.Stick.RIGHT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.InShootingRange;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Telemetry;

public class TeleopDriveCommand extends BaseDriveCommand {

    private final LightingSubsystem lighting;
    private final OperatorInput     oi;
    private final SlewRateLimiter   inputOmegaLimiter = new SlewRateLimiter(MAX_ANGULAR_VELOCITY_PCT_CHANGE_PER_CYCLE_FOR_TELEOP);
    private Rotation2d              headingSetpoint   = Rotation2d.fromDegrees(0);

    private boolean                 lockOnSpeaker     = false;

    /**
     * Used to drive a swerve robot in full field-centric mode.
     */
    public TeleopDriveCommand(SwerveSubsystem swerve, LightingSubsystem lighting, OperatorInput operatorInput) {
        super(swerve);
        this.lighting = lighting;
        this.oi       = operatorInput;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();

        // The FRC field-oriented ccoordinate system
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        final Alliance      alliance                     = getRunnymedeAlliance();

        // The coordinate system defines (0,0) as the right side of the blue alliance wall. The
        // x-axis is positive toward the red alliance, and the y-axis is positive to the left.
        // When the robot is on the red alliance, we need to invert inputs for the stick values
        // to move the robot in the right direction.
        final boolean       invert                       = alliance == Alliance.Red;

        // With the driver standing behind the driver station glass, "forward" on the left stick is
        // its y value, but that should convert into positive x movement on the field. The
        // Runnymede Controller inverts stick y-axis values, so "forward" on stick is positive.
        // Thus, positive y stick axis maps to positive x translation on the field.
        final double        vX                           = oi.getDriverControllerAxis(LEFT, Y);

        // Left and right movement on the left stick (the stick's x-axis) maps to the y-axis on the
        // field. Left on the stick (negative x) maps to positive y on the field, and vice versa.
        // Thus, negative x stick axis maps to positive y translation on the field.
        final double        vY                           = -oi.getDriverControllerAxis(LEFT, X);

        // Left and right on the right stick will change the direction the robot is facing - its
        // heading. Positive x values on the stick translate to clockwise motion, and vice versa.
        // The coordinate system has positive motion as CCW.
        // Therefore, negative x stick value maps to positive rotation on the field.
        final double        ccwRotAngularVelPct          = -oi.getDriverControllerAxis(RIGHT, X);

        // User wants to jump directly to a specific heading. Computation is deferred because it is
        // complex
        // and may not be necessary. See below for details.
        final int           rawDesiredHeadingDeg         = oi.getDriverPOV();

        final boolean       faceSpeaker                  = oi.isDriveFacingSpeaker();
        final Translation2d speaker                      = alliance == Alliance.Blue
            ? Constants.BotTarget.BLUE_SPEAKER.getLocation().toTranslation2d()
            : Constants.BotTarget.RED_SPEAKER.getLocation().toTranslation2d();

        final boolean       faceChain                    = oi.isDriveFacingChain();

        // Compute boost factor
        final boolean       isSlow                       = oi.isDriverLeftBumper();
        final boolean       isFast                       = oi.isDriverRightBumper();
        final double        boostFactor                  = isSlow ? SLOW_SPEED_FACTOR
            : (isFast ? MAX_SPEED_FACTOR : GENERAL_SPEED_FACTOR);


        Translation2d       velocity                     = calculateTeleopVelocity(vX, vY, boostFactor, invert);

        Rotation2d          omega;

        double              correctedCcwRotAngularVelPct = inputOmegaLimiter.calculate(ccwRotAngularVelPct);

        final String        modeForDebug;

        // User is steering!
        if (correctedCcwRotAngularVelPct != 0) {
            modeForDebug  = "Steering";
            // Compute omega
            lockOnSpeaker = false;
            double w = Math.pow(correctedCcwRotAngularVelPct, 3) * MAX_ROTATIONAL_VELOCITY_PER_SEC.getRadians();
            omega           = Rotation2d.fromRadians(w);
            // Save previous heading for when we are finished steering.
            headingSetpoint = swerve.getPose().getRotation();
        }
        else if (faceChain || rawDesiredHeadingDeg > -1) {
            modeForDebug  = faceChain ? "Jumping to Chain" : "Jumping to POV";
            lockOnSpeaker = false;
            final Rotation2d desiredHeading;
            if (faceChain) {
                // User wants to jump to a direction that faces the chain that is closest to the
                // robot
                desiredHeading = getChainHeading(swerve.getPose(), alliance);
            }
            else {
                // User wants to jump to POV
                // POV coordinates don't match field coordinates. POV is CW+ and field is CCW+.
                // Also,
                // POV 0 is 90 degrees on the field (for blue alliance, and -90 for red).
                // Invert and rotate as required.
                // BLUE field = MOD(-POV + 360, 360)
                // RED field = MOD(-POV + 180 + 360, 360)
                double correctedHeadingDeg = ((rawDesiredHeadingDeg * -1) + (invert ? 180 : 0) + 360) % 360;
                Telemetry.drive.teleop_correctedHeadingDeg = correctedHeadingDeg;
                desiredHeading                             = Rotation2d.fromDegrees(correctedHeadingDeg);
            }

            omega           = computeOmega(desiredHeading);
            // Save the previous heading for when the jump is done
            headingSetpoint = desiredHeading;
        }
        else if (faceSpeaker) {
            modeForDebug = "Face speaker";
            Rotation2d desiredHeading = super.getHeadingToFieldPosition(speaker)
                .plus(Rotation2d.fromDegrees(180));

            omega           = computeOmega(desiredHeading);
            headingSetpoint = desiredHeading;
            lockOnSpeaker   = true;
        }
        else {
            modeForDebug = "Translating";
            // Translating only. Just drive on the last heading we knew.

            if (lockOnSpeaker) {
                headingSetpoint = getHeadingToFieldPosition(speaker).plus(Rotation2d.fromDegrees(180));
            }
            else if (headingSetpoint == null) {
                headingSetpoint = swerve.getPose().getRotation();
            }

            omega = computeOmega(headingSetpoint);
        }

        if (lockOnSpeaker && Math.abs(distanceToFieldPosition(speaker)) < SPEAKER_SHOT_RANGE_METRES) {
            lighting.addPattern(SIGNAL, InShootingRange.getInstance());
        }
        else {
            lighting.removePattern(InShootingRange.class);
        }

        Telemetry.drive.teleop_vX                   = vX;
        Telemetry.drive.teleop_vY                   = vY;
        Telemetry.drive.teleop_ccwRotAngularVelPct  = ccwRotAngularVelPct;
        Telemetry.drive.teleop_rawDesiredHeadingDeg = rawDesiredHeadingDeg;
        Telemetry.drive.teleop_boostFactor          = boostFactor;
        Telemetry.drive.teleop_mode                 = modeForDebug;
        Telemetry.drive.teleop_lockOnSpeaker        = lockOnSpeaker;
        Telemetry.drive.teleop_velocity             = velocity;
        Telemetry.drive.teleop_theta                = headingSetpoint;
        Telemetry.drive.teleop_omega                = omega;

        swerve.driveFieldOriented(velocity, omega);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        headingSetpoint = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        super.isFinished();
        return false;

    }

    private static Translation2d calculateTeleopVelocity(double vX, double vY, double boostFactor, boolean invert) {
        // invert
        if (invert) {
            vX = -vX;
            vY = -vY;
        }

        // handy utilities
        Translation2d input     = new Translation2d(vX, vY);
        double        magnitude = input.getNorm();
        Rotation2d    angle     = input.getAngle();

        // handle case where in simulator, a value of 1,1 is possible whereas normally the
        // controller magnitude never exceeds 1
        magnitude = MathUtil.clamp(magnitude, -1, 1);

        // cube to allow more fine-grained control for user at low values
        magnitude = Math.pow(magnitude, 3);

        // convert from % to mps
        magnitude = magnitude * boostFactor * MAX_TRANSLATION_SPEED_MPS;

        // convert to vector
        return new Translation2d(magnitude, angle);
    }

    private static Rotation2d getChainHeading(Pose2d pose, Alliance alliance) {

        if (alliance == Alliance.Blue) {
            if (pose.getY() > Constants.FieldConstants.WING_LENGTH_METRES) {
                // center stage
                return Rotation2d.fromDegrees(180);
            }
            else {
                if (pose.getX() > Constants.FieldConstants.FIELD_WIDTH_METRES / 2) {
                    // stage left
                    return Rotation2d.fromDegrees(-60);
                }
                else {
                    // stage right
                    return Rotation2d.fromDegrees(60);
                }
            }
        }
        else {
            if (pose.getY() < Constants.FieldConstants.FIELD_LENGTH_METRES - Constants.FieldConstants.WING_LENGTH_METRES) {
                // center stage
                return Rotation2d.fromDegrees(180);
            }
            else {
                if (pose.getX() < Constants.FieldConstants.FIELD_WIDTH_METRES / 2) {
                    // stage left
                    return Rotation2d.fromDegrees(-60);
                }
                else {
                    // stage right
                    return Rotation2d.fromDegrees(60);
                }
            }
        }
    }
}
