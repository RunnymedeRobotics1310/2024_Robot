// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.Constants.Swerve.Chassis.TRACK_WIDTH_METRES;
import static frc.robot.Constants.Swerve.Chassis.WHEEL_BASE_METRES;
import static frc.robot.subsystems.vision.PoseConfidence.HIGH;
import static frc.robot.subsystems.vision.PoseConfidence.LOW;
import static frc.robot.subsystems.vision.PoseConfidence.MEDIUM;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.lighting.LightstripRegion;
import frc.robot.subsystems.lighting.pattern.Default;
import frc.robot.subsystems.lighting.pattern.VisionConfidenceNone;
import frc.robot.subsystems.vision.PoseConfidence;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OiConstants {

        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class Swerve {

        /**
         * Kill switch for the drive subsystem. Useful when testing other subsystems.
         */
        public static final boolean DISABLED = false;

        public static final class Chassis {

            /**
             * Front to back from the middle of the wheels
             */
            public static final double WHEEL_BASE_METRES            = inchesToMeters(24.75);
            /**
             * Side to side from the middle of the wheels
             */
            public static final double TRACK_WIDTH_METRES           = inchesToMeters(22.75);

            public static final double SDS_MK4I_WHEEL_RADIUS_METRES = 0.0051;

            /**
             * Specify the maximum speed a module can physically reach in m/s.
             * The SDS
             * <a href="https://www.swervedrivespecialties.com/products/mk4i-swerve-module">MK4i</a>
             * module with L2 gear ratio supports a maximum drive motor speed of 15.7ft/s (4.79m/s).
             *
             * Do not use this value in software to cap how fast the robot drives on the field.
             * For that, use {@link #MAX_TRANSLATION_SPEED_MPS}.
             */
            public static final double MAX_MODULE_SPEED_MPS         = 4.79;


            /*
             * ****************** Rotation Constants ******************
             */

            // todo: this needs to be increased
            public static final Rotation2d ROTATION_SLOW_ZONE                     = Rotation2d.fromDegrees(20);
            public static final Rotation2d MIN_ROTATIONAL_VELOCITY_PER_SEC        = Rotation2d.fromDegrees(10);
            public static final Rotation2d MAX_ROTATIONAL_VELOCITY_PER_SEC        = Rotation2d.fromDegrees(360);
            public static final double     MAX_ROTATION_ACCELERATION_RAD_PER_SEC2 = Rotation2d.fromRotations(1310).getRadians();
            public static final Rotation2d ROTATION_DECELERATION_DISTANCE         = Rotation2d.fromDegrees(5);
            public static final Rotation2d ROTATION_TOLERANCE                     = Rotation2d.fromDegrees(2);

            public static final class HeadingPIDConfig {
                // 0.4 is a little low but okay
                public static final double P = 0.4;
                public static final double I = 0;
                public static final double D = 0;
            }


            /*
             * ****************** Translation Constants ******************
             */

            /**
             * Set how fast you want the robot to actually translate across the field.
             * This is the "speed limit" of the robot.
             *
             * Practically speaking 4.42 m/s is a good max, but
             * consider 1-2 for development and 2-3 for competitions.
             */
            public static final double MAX_TRANSLATION_SPEED_MPS          = 4.42;
            public static final double TRANSLATION_TOLERANCE_METRES       = 0.02;
            public static final double DECEL_FROM_MAX_TO_STOP_DIST_METRES = 1.9;
            public static final double MAX_TRANSLATION_ACCELERATION_MPS2  = 8;

            /**
             * Standard drive speed factor. Regular teleop drive will use this factor of the max
             * translational speed.
             */
            public static final double GENERAL_SPEED_FACTOR               = .6;

            /**
             * Maximum drive speed factor. When boosting, this factor will be multiplied against the
             * max translational speed.
             * todo: tune
             */
            public static final double MAX_SPEED_FACTOR                   = 1;

            /**
             * Slow mode drive speed factor. When running in slow mode, this factor will be
             * multiplied against the max translational speed.
             * todo: tune
             */
            public static final double SLOW_SPEED_FACTOR                  = .1;

            public static final class VelocityPIDConfig {
                // public static final double P = 15;
                // public static final double P = 1.5;
                public static final double P = 1.2;
                // .002 is too low but stable
                public static final double I = 0;
                public static final double D = 0;
            }
        }

        public static final class Motor {
            public boolean            inverted;
            public int                currentLimitAmps;
            public double             nominalVoltage;
            public double             rampRate;
            public double             gearRatio;
            public double             p;
            public double             i;
            public double             d;
            public double             ff;
            public double             iz;
            public static final Motor DRIVE = new Motor();

            static {
                DRIVE.inverted         = true;
                DRIVE.currentLimitAmps = 40;
                DRIVE.nominalVoltage   = 12;
                DRIVE.rampRate         = 0.25;
                DRIVE.gearRatio        = 6.75; // SDS MK4i L2 --> 6.75:1
                DRIVE.p                = 0.11; // 0.0020645;
                DRIVE.i                = 0;
                DRIVE.d                = 0;
                DRIVE.ff               = 0;
                DRIVE.iz               = 0;
            }

            public static final Motor ANGLE = new Motor();

            static {
                ANGLE.inverted         = true;
                ANGLE.currentLimitAmps = 20;        // must not exceed 30 (fuse)
                ANGLE.nominalVoltage   = 12;
                ANGLE.rampRate         = 0.25;
                ANGLE.gearRatio        = 150.0 / 7; // SDS MK4i 150/7:1
                ANGLE.p                = 0.0125;    // 0.01
                ANGLE.i                = 0;
                ANGLE.d                = 0;
                ANGLE.ff               = 0;
                ANGLE.iz               = 0;
            }
        }

        public static final class Module {
            /**
             * The name of the module is used in debugging, but it is also used to
             * reference modules in the YAGSL config (if that implementation is used).
             * Be sure to give this name the same name as the yagsl config file (e.g.
             * src/main/deploy/swerve/neo/modules/backleft.json) --> "backleft"
             */
            public String              name;
            public double              wheelRadiusMetres;
            public Translation2d       locationMetres;
            public int                 driveCANID;
            public int                 angleCANID;
            public int                 encoderCANID;
            public double              encoderAbsoluteOffsetDegrees;
            public static final Module BACK_LEFT = new Module();

            static {
                BACK_LEFT.name                         = "backleft";
                BACK_LEFT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                BACK_LEFT.locationMetres               = new Translation2d(-TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2);
                BACK_LEFT.driveCANID                   = 35;
                BACK_LEFT.angleCANID                   = 36;
                BACK_LEFT.encoderCANID                 = 37;
                BACK_LEFT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.424316).getDegrees();
            }

            public static final Module BACK_RIGHT = new Module();

            static {
                BACK_RIGHT.name                         = "backright";
                BACK_RIGHT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                BACK_RIGHT.locationMetres               = new Translation2d(-TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2);
                BACK_RIGHT.driveCANID                   = 30;
                BACK_RIGHT.angleCANID                   = 31;
                BACK_RIGHT.encoderCANID                 = 32;
                BACK_RIGHT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.414551).getDegrees();
            }

            public static final Module FRONT_LEFT = new Module();

            static {
                FRONT_LEFT.name                         = "frontleft";
                FRONT_LEFT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                FRONT_LEFT.locationMetres               = new Translation2d(TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2);
                FRONT_LEFT.driveCANID                   = 10;
                FRONT_LEFT.angleCANID                   = 11;
                FRONT_LEFT.encoderCANID                 = 12;
                FRONT_LEFT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.094238).getDegrees();
            }

            public static final Module FRONT_RIGHT = new Module();

            static {
                FRONT_RIGHT.name                         = "frontright";
                FRONT_RIGHT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                FRONT_RIGHT.locationMetres               = new Translation2d(TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2);
                FRONT_RIGHT.driveCANID                   = 20;
                FRONT_RIGHT.angleCANID                   = 21;
                FRONT_RIGHT.encoderCANID                 = 22;
                FRONT_RIGHT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.533203).getDegrees();
            }
        }
    }

    public enum BotTarget {

        // Blue Field Targets
        BLUE_AMP(new Translation3d(1.8415, 8.2042, 0.873252)),
        BLUE_SOURCE(new Translation3d(15.632176, 0.564896, 0)),
        BLUE_SPEAKER(new Translation3d(0.0381, 5.547868, 2.124202)),
        BLUE_STAGE(new Translation3d(4.86791, 4.105656, 1.6764)),

        // Red Field Targets
        RED_AMP(new Translation3d(14.700758, 8.2042, 0.873252)),
        RED_SOURCE(new Translation3d(0.908812, 0.564769, 0)),
        RED_SPEAKER(new Translation3d(16.579342, 5.547868, 2.124202)),
        RED_STAGE(new Translation3d(11.676634, 4.105656, 1.6764)),

        // Blue Side Notes
        BLUE_NOTE_WOLVERINE(new Translation3d(2.9, 4.11, 0)),
        BLUE_NOTE_BARNUM(new Translation3d(2.9, 5.5, 0)),
        BLUE_NOTE_VALJEAN(new Translation3d(2.9, 7, 0)),

        // Red Side Notes
        RED_NOTE_WOLVERINE(new Translation3d(13.53, 4.11, 0)),
        RED_NOTE_BARNUM(new Translation3d(13.53, 5.5, 0)),
        RED_NOTE_VALJEAN(new Translation3d(13.53, 7, 0)),

        // Centre Field Notes
        CENTRE_NOTE_1(new Translation3d(8.16, 0.75, 0)),
        CENTRE_NOTE_2(new Translation3d(8.16, 2.43, 0)),
        CENTRE_NOTE_3(new Translation3d(8.16, 4.11, 0)),
        CENTRE_NOTE_4(new Translation3d(8.16, 5.79, 0)),
        CENTRE_NOTE_5(new Translation3d(8.16, 7.47, 0)),

        // When No Target is Set
        NONE(new Translation3d(0, 0, 0)),

        // No focus, but go to any tag visible
        ALL(new Translation3d(0, 0, 0));


        private final Translation3d location;

        BotTarget(Translation3d location) {
            this.location = location;
        }

        public Translation3d getLocation() {
            return location;
        }

        @Override
        public String toString() {
            return "BotTarget: " + name() + " at " + location;
        }
    }

    public static final class UsefulPoses {

        public static final Pose2d SCORE_BLUE_AMP        = (new Pose2d(BotTarget.BLUE_AMP.getLocation().getX(), 7.8,
            Rotation2d.fromDegrees(90)));
        public static final Pose2d SCORE_RED_AMP         = (new Pose2d(BotTarget.RED_AMP.getLocation().getX(), 7.8,
            Rotation2d.fromDegrees(90)));

        public static final Pose2d START_AT_BLUE_SPEAKER = new Pose2d(Constants.BotTarget.BLUE_SPEAKER.getLocation().getX(),
            1.6, new Rotation2d());
        public static final Pose2d START_AT_RED_SPEAKER  = new Pose2d(Constants.BotTarget.RED_SPEAKER.getLocation().getX(),
            16.54 - 1.6, new Rotation2d());

        public static final Pose2d BLUE_2_2_20           = new Pose2d(2, 2, Rotation2d.fromDegrees(20));
        public static final Pose2d RED_2_2_20            = new Pose2d(14.54, 2, Rotation2d.fromDegrees(-20));

    }

    public static final class FieldConstants {
        public static final double FIELD_WIDTH_METRES  = 16.54;
        public static final double FIELD_LENGTH_METRES = 8.02;

        public static final double WING_LENGTH_METRES  = 5.0;  // todo: confirm
    }

    public static final class VisionConstants {
        /** Time to switch pipelines and acquire a new vision target */
        public static final double  VISION_SWITCH_TIME_SEC         = .25;

        // todo: correct this
        public static Translation2d CAMERA_LOC_REL_TO_ROBOT_CENTER = new Translation2d(0, 30);

        /**
         * Utility method (STATIC) to map confidence and pose difference to a matrix of estimated
         * standard
         * deviations. The returned matrix values have been tuned based on the input and are not
         * dynamically calculated.
         *
         * @param confidence rating from the vision subsystem
         * @param poseDifferenceMetres difference between estimated pose and pose from vision
         * @return matrix of standard deviations, or null if values are too far out of bounds
         */
        public static Matrix<N3, N1> getVisionStandardDeviation(PoseConfidence confidence, double poseDifferenceMetres) {
            double xyMetresStds;
            double degreesStds;

            // todo: measure / tune these values
            if (confidence == HIGH) {
                xyMetresStds = 0.05;
                degreesStds  = 2;
            }
            else if (confidence == MEDIUM || poseDifferenceMetres < 0.5) {
                xyMetresStds = 0.15;
                degreesStds  = 6;
                // temporarily disable
                return null;
            }
            else if (confidence == LOW || poseDifferenceMetres < 0.8) {
                xyMetresStds = 0.30;
                degreesStds  = 12;
                // temporarily disable
                return null;
            }
            else { // Covers the Confidence.NONE case
                return null;
            }

            return VecBuilder.fill(xyMetresStds, xyMetresStds, Units.degreesToRadians(degreesStds));
        }
    }

    public static final class AutoConstants {

        public enum AutoPattern {
            DO_NOTHING,
            EXIT_ZONE, SCORE_1_AMP, SCORE_2_AMP, SCORE_2_5_AMP,
            SCORE_1_SPEAKER_STAY,
            SCORE_1_SPEAKER, SCORE_2_SPEAKER, SCORE_3_SPEAKER, SCORE_4_SPEAKER,
            PLAN_B
        }

        public enum Delay {
            NO_DELAY, WAIT_0_5_SECOND,
            WAIT_1_SECOND, WAIT_1_5_SECONDS,
            WAIT_2_SECONDS, WAIT_2_5_SECONDS,
            WAIT_3_SECONDS, WAIT_5_SECONDS
        }
    }

    public static final class LightingConstants {
        public static final int        LIGHT_STRING_PWM_PORT = 9;
        public static final int        LIGHT_STRIP_LENGTH    = 60;

        public static final Color      NOTE_ORANGE           = new Color(255, 20, 0);

        public static LightstripRegion VISPOSE               = new LightstripRegion(
            "Vision",
            0, 24,
            VisionConfidenceNone.class);
        public static LightstripRegion SIGNAL                = new LightstripRegion(
            "Signal",
            24, LIGHT_STRIP_LENGTH - 1,
            Default.class);
    }

    public static final class ShooterConstants {
        /**
         * The distance (metres) from which the shooter is likely to shoot and score on the speaker.
         * It may be possible to shoot from beyond this distance, but it would be with considerably
         * less reliability.
         */
        public static final double SPEAKER_SHOT_RANGE_METRES = 4.0;
    }


    public static class ArmPosition {

        public final double linkAngle;
        public final double aimAngle;

        public ArmPosition(double linkDegrees, double aimDegrees) {
            this.linkAngle = linkDegrees;
            this.aimAngle  = aimDegrees;
        }
    }

    public static final class ArmConstants {

        /**
         * Completely disable control over the link motor. Normally set to false,
         * but can be set to true when the link motor is not functioning correctly.
         */
        public static final boolean     DISABLE_LINK                       = false;
        /**
         * Completely disable control over the link motor. Normally set to false,
         * but can be set to true when the aim motor is not functioning correctly.
         */
        public static final boolean     DISABLE_AIM                        = false;

        public static final int         LINK_MOTOR_CAN_ADDRESS             = 40;
        public static final int         AIM_MOTOR_CAN_ADDRESS              = 41;

        public static final int         INTAKE_MOTOR_CAN_ADDRESS           = 50;
        public static final int         SHOOTER_MOTOR_CAN_ADDRESS          = 51;

        public static final int         LINK_ABSOLUTE_ENCODER_ANALOG_PORT  = 3;
        // Encoder constants to convert from Volts to Deg
        public static final double      LINK_ABSOLUTE_ENCODER_DEG_PER_VOLT = 49.11;
        public static final double      LINK_ABSOLUTE_ENCODER_OFFSET_DEG   = -12;


        public static final int         LINK_LOWER_LIMIT_SWITCH_DIO_PORT   = 0;

        public static final int         AIM_ABSOLUTE_ENCODER_ANALOG_PORT   = 2;
        // Encoder constants to convert from Volts to Deg
        public static final double      AIM_ABSOLUTE_ENCODER_DEG_PER_VOLT  = 43;
        // Increasing aim offset by 78 to account for a change that happened on Saturday Mar 9. New
        // measurement added March 10th at 11:20am
        // 73 degree offset on Mar 15.
        public static final double      AIM_ABSOLUTE_ENCODER_OFFSET_DEG    = 61.3 - 73;

        public static final int         INTAKE_NOTE_DETECTOR_DIO_PORT      = 1;

        /*
         * ARM PID CONTROLS
         */
        public static final double      AIM_PID_P                          = 0.02;

        /*
         * Key Arm Positions
         */
        // aim re-measured Mar 10, 2024 9:30am (was 35, set to 113) - diff - 78
        public static final ArmPosition COMPACT_ARM_POSITION               = new ArmPosition(185, 33);
        public static final ArmPosition INTAKE_ARM_POSITION                = new ArmPosition(116, 109);

        public static final ArmPosition OVER_INTAKE                        = new ArmPosition(134, 105);
        public static final ArmPosition NOTE_INTAKE_CLEARANCE_POSITION     = new ArmPosition(149, 144);

        // Transition position - over bumper
        public static final ArmPosition OVER_BUMPER_POSITION               = new ArmPosition(157, 76.89);
        // Transition position - above the lock position (arm not caught on stops)
        public static final ArmPosition UNLOCK_POSITION                    = new ArmPosition(200, 35);

        public static final ArmPosition SHOOT_SPEAKER_ARM_POSITION         = new ArmPosition(180, 90.0);    // Unfinished
        public static final ArmPosition SHOOT_SPEAKER_PODIUM_ARM_POSITION  = new ArmPosition(196, 42);

        // re-measured Mar 10, 2024 9:30am 2.4% arm (was 108, changed to 186; diff 78)
        public static final ArmPosition SHOOT_AMP_ARM_POSITION             = new ArmPosition(200, 108);
        public static final ArmPosition TRAP_ARM_POSITION                  = new ArmPosition(206.3, 102.92);

        public static final ArmPosition SOURCE_INTAKE_POSE                 = new ArmPosition(200, 35);


        // todo: fixme: indicate units in doc or constant name for all of these settings
        public static final double      FAST_AIM_SPEED                     = .7;
        public static final double      SLOW_AIM_SPEED                     = .1;
        public static final double      FAST_LINK_SPEED                    = .7;
        public static final double      SLOW_LINK_SPEED                    = .2;

        public static final double      SLOW_ARM_ZONE_DEG                  = 20.0;
        public static final double      AT_TARGET_DEG                      = 2;
        public static final double      DEFAULT_LINK_TOLERANCE_DEG         = 3;
        public static final double      DEFAULT_AIM_TOLERANCE_DEG          = 3;

        public static final double      INTAKE_INTAKE_SPEED                = .6;
        public static final double      INTAKE_REVERSE_SPEED               = -.3;

        public static final double      INTAKE_NOTE_REVERSAL_REVERSE_SPEED = -.075;

        public static final double      INTAKE_NOTE_REVERSAL_FORWARD_SPEED = .1;
        public static final long        INTAKE_SPINUP_WINDOW               = 500;

        public static final double      INTAKE_EJECT_INTAKE_SPEED          = -1.0;
        public static final double      INTAKE_EJECT_SHOOTER_SPEED         = -1.0;



        public static final double      SHOOTER_SPEAKER_SPEED              = 0.75;
        public static final double      SHOOTER_AMP_SPEED                  = 0.2;

        public static final double      LINK_MAX_DEGREES                   = 215;
        public static final double      LINK_MIN_DEGREES                   = 116;

        public static final double      AIM_MAX_DEGREES                    = 240;
        public static final double      AIM_MIN_DEGREES                    = 25;

        public static final double      ARM_MIN_ANGLE_SUM                  = 180;
        public static final double      ARM_MAX_ANGLE_SUM                  = 311;

        public static final double      AIM_X_SHOOTING                     = 0.254;
        public static final double      AIM_Y_SHOOTING                     = 0.61595;

        public static final double      SHOOTER_AIM_DIFFERENCE             = 47.9;

        /**
         * Amount of output required to hold the Aim Pivot when the Aim is parallel to the ground
         */
        public static final double      MAX_AIM_HOLD                       = 0.03;                          // 0.03;


        /**
         * Amount of output required to hold the Link Pivot when the Link and Aim are parallel to
         * the ground
         */
        public static final double      MAX_LINK_HOLD                      = 0.02;                          // 0.04;
    }

    public static final class ClimbConstants {

        public static final int        RIGHT_CLIMB_MOTOR_CAN_ADDRESS = 60;
        public static final int        LEFT_CLIMB_MOTOR_CAN_ADDRESS  = 61;

        public static final double     MAX_ROBOT_LIFT_SPEED          = 1;
        public static final double     RAISE_CLIMBERS_SPEED          = .75;
        public static final double     LOWER_CLIMBERS_SPEED          = .75;
        public static final double     INITIALIZE_CLIMBERS_SPEED     = 0.1;

        public static final double     CLIMB_MAX                     = 120;
        // encoder values

        public static final double     CLIMB_MIN                     = 3;

        public static final int        CLIMB_LIMIT_SWITCH_DIO_PORT_2 = 2;
        public static final int        CLIMB_LIMIT_SWITCH_DIO_PORT_3 = 3;

        // Slow Zones

        public static final double     SLOW_SPEED                    = 0.15;
        public static final int        BOTTOM_SLOW_ZONE              = 15;

        public static final int        TOP_SLOW_ZONE                 = 110;

        public static final double     METRES_PER_ENCODER_COUNT      = 0.6 / 120;

        public static final Rotation2d LEVEL_CLIMB_TOLERANCE         = Rotation2d.fromDegrees(3);
    }
}
