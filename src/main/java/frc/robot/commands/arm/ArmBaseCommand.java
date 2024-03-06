package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

public abstract class ArmBaseCommand extends LoggingCommand {

    ArmPosition                  targetArmPosition;

    protected final ArmSubsystem armSubsystem;

    public ArmBaseCommand(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    public void setArmTarget(ArmPosition armPosition) {
        targetArmPosition = armPosition;
    }


    public boolean driveToArmPosition(ArmPosition targetArmPosition, double targetAngleTolerance) {
        // todo: fixme: set tolerances as a constants in ArmConstants as a Rotation2d to provide
        // clarity into units, or else name parameters to hint at units
        return driveToArmPosition(targetArmPosition.linkAngle, targetArmPosition.aimAngle, targetAngleTolerance);
    }

    public boolean driveToArmPosition(double targetLinkAngle, double targetAimAngle, double targetAngleTolerance) {

        double currentLinkAngle = armSubsystem.getLinkAngle();
        double currentAimAngle  = armSubsystem.getAimAngle();

        // The angle tolerance cannot be set less than the AT_TARGET constant
        targetAngleTolerance = Math.max(ArmConstants.AT_TARGET_DEG, targetAngleTolerance);

        // Calculate the errors

        double linkAngleError = targetLinkAngle - currentLinkAngle;
        double aimAngleError  = targetAimAngle - currentAimAngle;

        double linkSpeed      = 0;
        double aimSpeed       = 0;

        // Move the motor with the larger error at the appropriate speed (Fast or Slow)
        // depending on the error

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

        if (aimAngleError < 0) {
            aimSpeed *= -1.0;
        }

        if (linkAngleError < 0) {
            linkSpeed *= -1.0;
        }

        // Adjust the output speeds by compensating for gravity.
        aimSpeed  = aimSpeed + calcAimHold(currentAimAngle, currentLinkAngle);
        linkSpeed = linkSpeed + calcLinkHold(currentAimAngle, currentLinkAngle);

        // Set the motor speeds

        armSubsystem.setLinkPivotSpeed(linkSpeed);
        armSubsystem.setAimPivotSpeed(aimSpeed);

        // Determine if the arm is within the requested range
        if (Math.abs(linkAngleError) <= targetAngleTolerance
            && Math.abs(aimAngleError) <= targetAngleTolerance) {

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

        return Math.sin(linkAngleRad) + Math.sin(aimContributionRad) / 2.0;
    }

}
