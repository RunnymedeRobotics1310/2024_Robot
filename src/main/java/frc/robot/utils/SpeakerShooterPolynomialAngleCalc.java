package frc.robot.utils;

public class SpeakerShooterPolynomialAngleCalc {

    public static final double  MIN_DISTANCE      = 1.6;
    public static final double  MAX_DISTANCE      = 3.9;
    public static final double  BONUS_DISTANCE    = 5.5;

    // Below this, use the a polynomial, above this, use the b polynomial
    private static final double BOUNDARY_DISTANCE = 3.9;

    // Coefficients of the polynomial for 1.6m <--> 3.9m
    private static final double a0                = -318.286;
    private static final double a1                = 682.8;
    private static final double a2                = -538.458;
    private static final double a3                = 212.001;
    private static final double a4                = -40.7422;
    private static final double a5                = 3.04426;

    // Coefficients of the polynomial for 3.9m <--> 5.5m
    private static final double b0                = 37.9745;
    private static final double b1                = 3.39394;

    private static double calculateAimAngleLowerBound(double distance) {
        // Calculate the aim angle using the polynomial regression model
        return a0
            + (a1 * distance)
            + (a2 * Math.pow(distance, 2))
            + (a3 * Math.pow(distance, 3))
            + (a4 * Math.pow(distance, 4))
            + (a5 * Math.pow(distance, 5));
    }

    private static double calculateAimAngleUpperBound(double distance) {
        // Calculate the aim angle using the polynomial regression model
        return b0
            + (b1 * distance);
    }

    /**
     * Calculates aim angle based on quadratic equation fit for the following data:
     *
     * @param distance Distance in meters to target
     * @return aim angle in degrees
     */
    public static double calculateAimAngle(double distance) {
        if (distance < BOUNDARY_DISTANCE) {
            return calculateAimAngleLowerBound(distance);
        }
        else {
            return calculateAimAngleUpperBound(distance);
        }
    }

}
