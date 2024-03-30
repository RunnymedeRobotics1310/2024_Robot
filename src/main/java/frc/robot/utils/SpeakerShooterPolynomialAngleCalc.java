package frc.robot.utils;

public class SpeakerShooterPolynomialAngleCalc {
    // Coefficients of the polynomial
    private static final double a0 = -318.286;
    private static final double a1 = 682.8;
    private static final double a2 = -538.458;
    private static final double a3 = 212.001;
    private static final double a4 = -40.7422;
    private static final double a5 = 3.04426;

    /**
     * Calculates aim angle based on quadratic equation fit for the following data:
     *
     * @param distance Distance in meters to target
     * @return aim angle in degrees
     */
    public static double calculateAimAngle(double distance) {
        // Calculate the aim angle using the polynomial regression model
        return a0
                + (a1 * distance)
                + (a2 * Math.pow(distance, 2))
                + (a3 * Math.pow(distance, 3))
                + (a4 * Math.pow(distance, 4))
                + (a5 * Math.pow(distance, 5));
    }

}
