package frc.robot.subsystems;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.DoubleUnaryOperator;

/**
 * Time-of-Flight functions for use with the SOTM solvers.
 *
 * Pass the returned DoubleUnaryOperator to solver.solve(..., tofFunc, ...).
 */
public class TofFunctions {

    /**
     * Simple physics-based TOF: flat-ground projectile, no drag.
     *
     *     tof = distance / (v0 * cos(theta))
     *
     * @param launchSpeed     total launch speed (m/s), e.g. 12.0
     * @param launchAngleDeg  launch angle above horizontal (degrees), e.g. 40.0
     */
    public static DoubleUnaryOperator ballistic(double launchSpeed, double launchAngleDeg) {
        double vHorizontal = launchSpeed * Math.cos(Math.toRadians(launchAngleDeg));
        if (vHorizontal <= 0) return distance -> -1.0;
        return distance -> distance / vHorizontal;
    }

    /**
     * Interpolating lookup table TOF. Measured empirically on your robot.
     *
     * @param data  map of distance (m) -> measured TOF (s)
     */
    public static DoubleUnaryOperator lookup(Map<Double, Double> data) {
        TreeMap<Double, Double> table = new TreeMap<>(data);
        return distance -> {
            if (table.isEmpty()) throw new IllegalStateException("Empty TOF table");

            Double lo = table.floorKey(distance);
            Double hi = table.ceilingKey(distance);

            if (lo == null) return table.firstEntry().getValue();
            if (hi == null) return table.lastEntry().getValue();
            if (lo.equals(hi)) return table.get(lo);

            double loVal = table.get(lo);
            double hiVal = table.get(hi);
            double t = (distance - lo) / (hi - lo);
            return loVal + t * (hiVal - loVal);
        };
    }

    /**
     * Generates a lookup table from a wheel speed function.
     * Samples the function at evenly spaced distances and computes ballistic TOF at each.
     *
     * @param wheelSpeedFunc  distance (m) -> exit velocity (m/s)
     * @param launchAngleDeg  launch angle above horizontal (degrees)
     * @param minDist         minimum sampling distance (m)
     * @param maxDist         maximum sampling distance (m)
     * @param numSamples      number of sample points
     */
    public static DoubleUnaryOperator fromWheelSpeed(
            DoubleUnaryOperator wheelSpeedFunc,
            double launchAngleDeg,
            double minDist,
            double maxDist,
            int numSamples) {

        double cosTheta = Math.cos(Math.toRadians(launchAngleDeg));
        TreeMap<Double, Double> table = new TreeMap<>();
        double step = (maxDist - minDist) / (numSamples - 1);

        for (int i = 0; i < numSamples; i++) {
            double dist = minDist + step * i;
            double vH = wheelSpeedFunc.applyAsDouble(dist) * cosTheta;
            double tof = (vH <= 0) ? -1.0 : dist / vH;
            table.put(dist, tof);
        }

        return lookup(table);
    }

    /**
     * Quadratic regression TOF: fits y = ax^2 + bx + c to the data using least-squares.
     *
     * @param data  map of distance (m) -> measured TOF (s)
     */
    public static DoubleUnaryOperator quadratic_regression(Map<Double, Double> data) {
        int n = data.size();
        if (n < 3) throw new IllegalArgumentException("Need at least 3 data points for quadratic fit");

        // Build normal equations for least-squares: y = a*x^2 + b*x + c
        double sumX0 = 0, sumX1 = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
        double sumY = 0, sumXY = 0, sumX2Y = 0;

        for (Map.Entry<Double, Double> entry : data.entrySet()) {
            double x = entry.getKey();
            double y = entry.getValue();
            double x2 = x * x;
            sumX0 += 1;
            sumX1 += x;
            sumX2 += x2;
            sumX3 += x2 * x;
            sumX4 += x2 * x2;
            sumY  += y;
            sumXY += x * y;
            sumX2Y += x2 * y;
        }

        // Solve 3x3 system via Cramer's rule:
        //   | sumX4  sumX3  sumX2 | | a |   | sumX2Y |
        //   | sumX3  sumX2  sumX1 | | b | = | sumXY  |
        //   | sumX2  sumX1  sumX0 | | c |   | sumY   |
        double det = sumX4 * (sumX2 * sumX0 - sumX1 * sumX1)
                   - sumX3 * (sumX3 * sumX0 - sumX1 * sumX2)
                   + sumX2 * (sumX3 * sumX1 - sumX2 * sumX2);

        if (Math.abs(det) < 1e-12) throw new IllegalStateException("Singular matrix in quadratic fit");

        double a = (sumX2Y * (sumX2 * sumX0 - sumX1 * sumX1)
                  - sumXY  * (sumX3 * sumX0 - sumX1 * sumX2)
                  + sumY   * (sumX3 * sumX1 - sumX2 * sumX2)) / det;

        double b = (sumX4 * (sumXY * sumX0 - sumY * sumX1)
                  - sumX3 * (sumX2Y * sumX0 - sumY * sumX2)
                  + sumX2 * (sumX2Y * sumX1 - sumXY * sumX2)) / det;

        double c = (sumX4 * (sumX2 * sumY - sumX1 * sumXY)
                  - sumX3 * (sumX3 * sumY - sumX1 * sumX2Y)
                  + sumX2 * (sumX3 * sumXY - sumX2 * sumX2Y)) / det;

        return distance -> a * distance * distance + b * distance + c;
    }

    /**
     * Example lookup table with placeholder data. Replace with your robot's measurements.
     */
    public static DoubleUnaryOperator exampleLookup() {
        return lookup(Map.ofEntries(
            Map.entry(1.0, 0.22), Map.entry(1.5, 0.30), Map.entry(2.0, 0.38),
            Map.entry(2.5, 0.45), Map.entry(3.0, 0.52), Map.entry(3.5, 0.58),
            Map.entry(4.0, 0.65), Map.entry(4.5, 0.72), Map.entry(5.0, 0.78),
            Map.entry(6.0, 0.92), Map.entry(7.0, 1.05)
        ));
    }

    /**
     * Example quadratic regression from measured data.
     * Fitted: y = 0.0184x^2 + 0.0719x + 0.5907  (R^2 = 0.7924)
     */
    public static DoubleUnaryOperator exampleQuadratic() {
        return quadratic_regression(Map.ofEntries(
            Map.entry(3.77, 1.21),
            Map.entry(3.27, 1.16),
            Map.entry(2.12, 0.83),
            Map.entry(4.30, 1.23),
            Map.entry(2.29, 0.91)
        ));
    }
}
