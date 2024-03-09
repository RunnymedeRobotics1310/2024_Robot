package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

public abstract class ArmBaseCommand extends LoggingCommand {

    private enum Direction {
        UP, DOWN
    }

    ArmPosition                  targetArmPosition;

    protected final ArmSubsystem armSubsystem;

    Direction                    linkTransitionDirection   = Direction.UP;
    Direction                    aimTransitionDirection    = Direction.DOWN;

    long                         transitionStartTimeMillis = 0;

    public ArmBaseCommand(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    public void setArmTarget(ArmPosition armPosition) {
        targetArmPosition = armPosition;
    }


    /**
     * Drive through arm position
     *
     * This command optimizes the arm transition through the selected position. It will not back up
     * to try
     * to correct to the precise position, but may return true after the angles are further past the
     * target
     * than the starting position.
     *
     * @param targetArmPosition containing link and aim angles.
     * @param targetAngleTolerance in degrees, which is used to determine whether the arm is past
     * the target
     * @return {@code true} if both the link and aim angles are within the tolerance degrees of the
     * target, {@code false}
     * otherwise
     */
    public boolean driveThroughArmPosition(ArmPosition targetArmPosition, double linkTolerance, double aimTolerance) {
        return driveThroughArmPosition(targetArmPosition.linkAngle, targetArmPosition.aimAngle, linkTolerance, aimTolerance);
    }

    /**
     * Drive through arm position
     *
     * This command optimizes the arm transition through the selected position. It will not back up
     * to try
     * to correct to the precise position, and may return true after the angles are further past the
     * target
     * than the starting position and the angles may be outside the target tolerance.
     *
     * @param targetLinkAngle in degrees
     * @param targetAimAngle in degrees
     * @param targetAngleTolerance in degrees, which is used to determine whether the arm is past
     * the target
     * @return {@code true} if both the link and aim angles are within the tolerance degrees of the
     * target, {@code false}
     * otherwise
     */
    public boolean driveThroughArmPosition(double targetLinkAngle, double targetAimAngle, double linkTolerance,
        double aimTolerance) {

        double currentLinkAngle = armSubsystem.getLinkAngle();
        double currentAimAngle  = armSubsystem.getAimAngle();

        // The angle tolerance cannot be set less than the AT_TARGET constant
        linkTolerance = Math.max(ArmConstants.AT_TARGET_DEG, linkTolerance);
        aimTolerance  = Math.max(ArmConstants.AT_TARGET_DEG, aimTolerance);

        // If the target has changed, determine whether to move the link and aim up or down
        if (targetArmPosition == null || targetLinkAngle != targetArmPosition.linkAngle
            || targetAimAngle != targetArmPosition.aimAngle) {

            targetArmPosition = new ArmPosition(targetLinkAngle, targetAimAngle);

            if (targetAimAngle > currentAimAngle) {
                aimTransitionDirection = Direction.UP;
            }
            else {
                aimTransitionDirection = Direction.DOWN;
            }

            if (targetLinkAngle > currentLinkAngle) {
                linkTransitionDirection = Direction.UP;
            }
            else {
                linkTransitionDirection = Direction.DOWN;
            }

            transitionStartTimeMillis = System.currentTimeMillis();

            log("Transition through angle : link " + targetLinkAngle + ", " + linkTransitionDirection + ", link tolerance "
                + linkTolerance
                + ", aim " + targetAimAngle + ", " + aimTransitionDirection + " aim tolerance " + aimTolerance + "deg");
        }

        /*
         * Calculate the errors
         *
         * There is no error if past the transition point.
         */
        double linkAngleError = targetLinkAngle - currentLinkAngle;

        if (linkTransitionDirection == Direction.UP) {
            linkAngleError = Math.max(0, linkAngleError);
        }
        else {
            linkAngleError = Math.min(linkAngleError, 0);
        }


        double aimAngleError = targetAimAngle - currentAimAngle;

        if (aimTransitionDirection == Direction.UP) {
            aimAngleError = Math.max(0, aimAngleError);
        }
        else {
            aimAngleError = Math.min(aimAngleError, 0);
        }

        // Determine the relative motor speeds
        double linkSpeed = 0;
        double aimSpeed  = 0;

        if (Math.abs(aimAngleError) >= Math.abs(linkAngleError)) {

            aimSpeed = ArmConstants.FAST_AIM_SPEED;

            if (Math.abs(aimAngleError) <= ArmConstants.SLOW_ARM_ZONE_DEG) {
                aimSpeed = ArmConstants.SLOW_AIM_SPEED;
            }

            if (Math.abs(aimAngleError) <= ArmConstants.AT_TARGET_DEG) {
                aimSpeed = 0;
            }

            // Set the link speed relative to the aim speed.
            linkSpeed = Math.abs(linkAngleError / aimAngleError) * aimSpeed;

            if (Math.abs(linkAngleError) <= ArmConstants.SLOW_ARM_ZONE_DEG) {
                linkSpeed = Math.min(linkSpeed, ArmConstants.SLOW_LINK_SPEED);
            }

            if (Math.abs(linkAngleError) <= ArmConstants.AT_TARGET_DEG) {
                linkSpeed = 0;
            }
        }
        else {

            // Link error is larger than Aim error
            // Set the link speed to the maximum

            linkSpeed = ArmConstants.FAST_LINK_SPEED;

            if (Math.abs(linkAngleError) <= ArmConstants.SLOW_ARM_ZONE_DEG) {
                linkSpeed = ArmConstants.SLOW_LINK_SPEED;
            }

            if (Math.abs(linkAngleError) <= ArmConstants.AT_TARGET_DEG) {
                linkSpeed = 0;
            }

            // Set the link speed relative to the aim speed.
            aimSpeed = Math.abs(aimAngleError / linkAngleError) * linkSpeed;

            if (Math.abs(aimAngleError) <= ArmConstants.SLOW_ARM_ZONE_DEG) {
                aimSpeed = Math.min(aimSpeed, ArmConstants.SLOW_AIM_SPEED);
            }

            if (Math.abs(aimAngleError) <= ArmConstants.AT_TARGET_DEG) {
                aimSpeed = 0;
            }
        }

        if (linkAngleError < 0) {
            linkSpeed *= -1.0;
        }

        if (aimAngleError < 0) {
            aimSpeed *= -1.0;
        }

        // Prioritize the UP speeds by adding .1 to the up speed and delaying down for
        // for .1 seconds
        if (linkSpeed > 0) {
            linkSpeed += .1;
        }

        if (aimSpeed > 0) {
            aimSpeed += .1;
        }

        // delay the down directions
        if (System.currentTimeMillis() - transitionStartTimeMillis < 100) {
            linkSpeed = Math.max(0, linkSpeed);
            aimSpeed  = Math.max(0, aimSpeed);
        }

        // Adjust the output speeds by compensating for gravity.
        linkSpeed = linkSpeed + calcLinkHold(currentAimAngle, currentLinkAngle);
        aimSpeed  = aimSpeed + calcAimHold(currentAimAngle, currentLinkAngle);

        // Set the motor speeds

        armSubsystem.setLinkPivotSpeed(linkSpeed);
        armSubsystem.setAimPivotSpeed(aimSpeed);

        /*
         * Determine if we are at or past the target
         */
        boolean linkAtTarget = false;
        if (linkTransitionDirection == Direction.UP) {
            if (currentLinkAngle >= targetLinkAngle - linkTolerance) {
                linkAtTarget = true;
            }
        }
        else {
            if (currentLinkAngle <= targetLinkAngle + linkTolerance) {
                linkAtTarget = true;
            }
        }

        boolean aimAtTarget = false;
        if (aimTransitionDirection == Direction.UP) {
            if (currentAimAngle >= targetAimAngle - aimTolerance) {
                aimAtTarget = true;
            }
        }
        else {
            if (currentAimAngle <= targetAimAngle + aimTolerance) {
                aimAtTarget = true;
            }
        }

        return linkAtTarget && aimAtTarget;
    }

    /**
     * Drive to arm position
     *
     * @param targetArmPosition containing link and aim angles
     * @param targetAngleTolerance in degrees, which is used to determine whether the arm is at the
     * target
     * @return {@code true} if both the link and aim angles are within the tolerance degrees of the
     * target, {@code false}
     * otherwise
     */
    public boolean driveToArmPosition(ArmPosition targetArmPosition, double linkTolerance, double aimTolerance) {
        // todo: fixme: set tolerances as a constants in ArmConstants as a Rotation2d to provide
        // clarity into units, or else name parameters to hint at units
        return driveToArmPosition(targetArmPosition.linkAngle, targetArmPosition.aimAngle, linkTolerance, aimTolerance);
    }

    /**
     * Drive to arm position
     *
     * @param targetLinkAngle in degrees
     * @param targetAimAngle in degrees
     * @param targetAngleTolerance in degrees, which is used to determine whether the arm is at the
     * target
     * @return {@code true} if both the link and aim angles are within the tolerance degrees of the
     * target, {@code false}
     * otherwise
     */
    public boolean driveToArmPosition(double targetLinkAngle, double targetAimAngle, double linkTolerance, double aimTolerance) {

        double currentLinkAngle = armSubsystem.getLinkAngle();
        double currentAimAngle  = armSubsystem.getAimAngle();

        // The angle tolerance cannot be set less than the AT_TARGET constant
        linkTolerance     = Math.max(ArmConstants.AT_TARGET_DEG, linkTolerance);
        aimTolerance      = Math.max(ArmConstants.AT_TARGET_DEG, aimTolerance);

        targetArmPosition = new ArmPosition(targetLinkAngle, targetAimAngle);

        // Calculate the errors

        double linkAngleError = targetLinkAngle - currentLinkAngle;
        double aimAngleError  = targetAimAngle - currentAimAngle;

        double linkSpeed      = 0;
        double aimSpeed       = 0;

        // Move the motor with the larger error at the appropriate speed (Fast or Slow)
        // depending on the error

        // if (Math.abs(aimAngleError) >= Math.abs(linkAngleError)) {
        if (linkAngleError > 0 || aimAngleError > 0) {

            aimSpeed = ArmConstants.FAST_AIM_SPEED;

            if (Math.abs(aimAngleError) <= ArmConstants.SLOW_ARM_ZONE_DEG) {
                aimSpeed = ArmConstants.SLOW_AIM_SPEED;
            }

            if (Math.abs(aimAngleError) <= ArmConstants.AT_TARGET_DEG) {
                aimSpeed = 0;
            }

            // Set the link speed relative to the aim speed.
            linkSpeed = Math.abs(linkAngleError / aimAngleError) * aimSpeed;

            if (Math.abs(linkAngleError) <= ArmConstants.SLOW_ARM_ZONE_DEG) {
                linkSpeed = Math.min(linkSpeed, ArmConstants.SLOW_LINK_SPEED);
            }

            if (Math.abs(linkAngleError) <= ArmConstants.AT_TARGET_DEG) {
                linkSpeed = 0;
            }
        }
        else {

            // Link error is larger than Aim error
            // Set the link speed to the maximum

            linkSpeed = ArmConstants.FAST_LINK_SPEED;

            if (Math.abs(linkAngleError) <= ArmConstants.SLOW_ARM_ZONE_DEG) {
                linkSpeed = ArmConstants.SLOW_LINK_SPEED;
            }

            if (Math.abs(linkAngleError) <= ArmConstants.AT_TARGET_DEG) {
                linkSpeed = 0;
            }

            // Set the link speed relative to the aim speed.
            aimSpeed = Math.abs(aimAngleError / linkAngleError) * linkSpeed;

            if (Math.abs(aimAngleError) <= ArmConstants.SLOW_ARM_ZONE_DEG) {
                aimSpeed = Math.min(aimSpeed, ArmConstants.SLOW_AIM_SPEED);
            }

            if (Math.abs(aimAngleError) <= ArmConstants.AT_TARGET_DEG) {
                aimSpeed = 0;
            }
        }

        if (aimAngleError < 0) {
            aimSpeed *= -1.0;
        }

        if (linkAngleError < 0) {
            linkSpeed *= -1.0;
        }

        aimSpeed  = calcAimOmega(targetAimAngle, aimTolerance);


        // Adjust the output speeds by compensating for gravity.
        aimSpeed  = aimSpeed + calcAimHold(currentAimAngle, currentLinkAngle);
        linkSpeed = linkSpeed + calcLinkHold(currentAimAngle, currentLinkAngle);

        // Set the motor speeds

        armSubsystem.setLinkPivotSpeed(linkSpeed);
        armSubsystem.setAimPivotSpeed(aimSpeed);

        // Determine if the arm is within the requested range
        if (Math.abs(linkAngleError) <= linkTolerance
            && Math.abs(aimAngleError) <= aimTolerance) {

            return true;
        }

        return false;
    }

    /**
     * Calculate the current required to hold the aim stationary at these angles.
     * <p>
     * Since the aim angle is measured relative to the link, both angles are required in order to
     * calculate the position of the arm relative to the force of gravity.
     *
     * @param aimAngle
     * @param linkAngle
     * @return hold motor output
     */
    private double calcAimHold(double aimAngle, double linkAngle) {

        /*
         * The torque on the aim pivot point depends on the angle of the aim
         * relative to the ground.
         *
         * The aim angle is relative to the link, and the link is relative to the ground.
         *
         * Link Angles vs Torque
         * Link = 90 - link parallel to the floor - max torque on the link joint
         * Link = 180 - link straight up - minimum torque on the link joint
         *
         * Aim Angles vs Torque (depend on link angle)
         * Link = 90 and Aim = 180 - link and aim parallel to the floor - max torque on the link
         * joint
         * Link = 90 and Aim = 270 - link parallel to the floor - aim straight up - no torque on the
         * aim joint
         * (note this is not a valid arm position ^^^)
         *
         * Link = 180 and Aim = 90 - link straight up, aim paralled to the floor - max torque on the
         * aim joint
         *
         */

        double aimTorqueAngle    = aimAngle + linkAngle - 180;

        // Convert to radians 2pi rad = 360 deg

        double aimTorqueAngleRad = Math.PI * aimTorqueAngle / 180.0;

        // Multiply the hold current by the cosine of the torque angle
        // This applies a full hold at 90 deg, zero hold at 180,
        // and negative hold at angles greater than 180.
        return ArmConstants.MAX_AIM_HOLD * Math.sin(aimTorqueAngleRad);

    }

    /**
     * Calculate the current required to hold the link stationary at these angles.
     * <p>
     * A link angle of 90 degrees means the link will be parallel to the ground
     * creating maximum torque on the link.
     * calculate the position of the arm relative to the force of gravity.
     *
     * @param aimAngle
     * @param linkAngle
     * @return hold motor output
     */
    private double calcLinkHold(double aimAngle, double linkAngle) {

        /*
         * The torque on the link pivot point depends on the angle of the link, and the
         * position of the aim relative to the ground.
         *
         * Assume that the Center of Mass is about a link distance out onto the aim
         *
         * Calculate the effective x-horizontal distance from the link, and
         * multiply the maximum torque by a ratio of the effective distance.
         *
         * Link = 90 - link parallel to the floor: Aim = 180 - also parallel to the floor = max
         * torque on the link joint
         */

        double linkAngleRad         = Math.PI * linkAngle / 180.0;

        double aimContributionAngle = linkAngle + aimAngle - 180;

        double aimContributionRad   = Math.PI * aimContributionAngle;

        return ArmConstants.MAX_LINK_HOLD * (Math.sin(linkAngleRad) + Math.sin(aimContributionRad) / 2.0);
    }


    public double calcAimOmega(double targetAngle, double tolerance) {

        double kP           = ArmConstants.AIM_PID_P;
        double currentAngle = armSubsystem.getAimAngle();

        double error        = targetAngle - currentAngle;

        if (Math.abs(error) < tolerance) {
            return 0;
        }

        return error * kP;
    }
}
