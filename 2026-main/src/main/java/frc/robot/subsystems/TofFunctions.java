
package frc.robot.subsystems;

import java.util.TreeMap;
import java.util.Map;
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
}
