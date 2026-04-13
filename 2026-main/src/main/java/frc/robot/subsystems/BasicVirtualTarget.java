package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

/**
 * Method 1: Basic Virtual Target
 *
 * First-order only: offset = -v * tof, iterated until convergence.
 * Ignores acceleration entirely.
 */
public class BasicVirtualTarget {

    private final int iterations;

    public BasicVirtualTarget(int iterations) {
        this.iterations = iterations;
    }

    public BasicVirtualTarget() {
        this(10);
    }

    /**
     * Compute the virtual target offset vector.
     *
     * @param robotPos   {x, y} field-relative position (m)
     * @param robotVel   {vx, vy} field-relative velocity (m/s)
     * @param targetPos  {x, y} field-relative target position (m)
     * @param tofFunc    distance (m) -> time-of-flight (s)
     * @param latency    system latency to compensate (s)
     * @return {x, y} virtual target position
     */
    public double[] solve(
            double[] robotPos,
            double[] robotVel,
            double[] targetPos,
            DoubleUnaryOperator tofFunc,
            double latency) {

        double posX = robotPos[0] + robotVel[0] * latency;
        double posY = robotPos[1] + robotVel[1] * latency;

        double vtX = targetPos[0];
        double vtY = targetPos[1];
        double tof = 0;

        for (int i = 0; i < iterations; i++) {
            double dx = vtX - posX;
            double dy = vtY - posY;
            tof = tofFunc.applyAsDouble(Math.sqrt(dx * dx + dy * dy));
            vtX = targetPos[0] - robotVel[0] * tof;
            vtY = targetPos[1] - robotVel[1] * tof;
        }

        return new double[] { vtX, vtY };
    }
}
