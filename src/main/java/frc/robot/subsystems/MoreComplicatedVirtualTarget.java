package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

/**
 * Virtual Target with 2nd-order (acceleration) correction.
 *
 * Same iterative approach as BasicVirtualTarget but accounts for
 * robot acceleration during the ball's flight:
 *     offset = -v*tof - 0.5*a*tof^2
 */
public class MoreComplicatedVirtualTarget {

    private final int iterations;
    // sensitivity scalars: {velX, velY, omega}. 1.0 = full correction, 0.0 = no correction
    private double[] sensitivity = {1.0, 1.0, 1.0};

    public MoreComplicatedVirtualTarget(int iterations) {
        this.iterations = iterations;
    }

    public MoreComplicatedVirtualTarget() {
        this(10);
    }

    /** Set sensitivity for {velX, velY, omega}. 1.0 = default, <1 = less correction. */
    public void setSensitivity(double velXScale, double velYScale, double omegaScale) {
        this.sensitivity = new double[] {velXScale, velYScale, omegaScale};
    }

    /**
     * @param robotPos       {x, y} field-relative position (m)
     * @param robotVel       {vx, vy} field-relative velocity (m/s)
     * @param robotAcc       {ax, ay} field-relative acceleration (m/s^2)
     * @param targetPos      {x, y} field-relative target position (m)
     * @param tofFunc        distance (m) -> time-of-flight (s)
     * @param latency        system latency to compensate (s)
     * @param deltaZ         target z - robot z (m), constant
     * @param omega          robot angular velocity (rad/s), positive = CCW
     * @return {x, y} virtual target position
     */
    public double[] solve(
            double[] robotPos,
            double[] robotVel,
            double[] robotAcc,
            double[] targetPos,
            DoubleUnaryOperator tofFunc,
            double latency,
            double deltaZ,
            double omega) {
        return solve(robotPos, robotVel, robotAcc, targetPos, tofFunc, latency, deltaZ, omega, new double[] {0, 0});
    }

    public double[] solve(
            double[] robotPos,
            double[] robotVel,
            double[] robotAcc,
            double[] targetPos,
            DoubleUnaryOperator tofFunc,
            double latency,
            double deltaZ,
            double omega,
            double[] shooterOffset) {

        // apply sensitivity scaling
        double scaledOmega = omega * sensitivity[2];
        double velX = robotVel[0] * sensitivity[0];
        double velY = robotVel[1] * sensitivity[1];
        if (shooterOffset != null && scaledOmega != 0) {
            velX += -scaledOmega * shooterOffset[1];
            velY +=  scaledOmega * shooterOffset[0];
        }

        // 2nd-order latency compensation
        double posX = robotPos[0] + velX * latency + 0.5 * robotAcc[0] * latency * latency;
        double posY = robotPos[1] + velY * latency + 0.5 * robotAcc[1] * latency * latency;
        velX += robotAcc[0] * latency;
        velY += robotAcc[1] * latency;
        double accX = robotAcc[0];
        double accY = robotAcc[1];

        double dz2 = deltaZ * deltaZ;

        double dx0 = targetPos[0] - posX;
        double dy0 = targetPos[1] - posY;
        double tof0 = tofFunc.applyAsDouble(Math.sqrt(dx0 * dx0 + dy0 * dy0 + dz2));
        if (!Double.isFinite(tof0) || tof0 <= 0) {
            return new double[] { targetPos[0], targetPos[1] };
        }

        double vtX = targetPos[0];
        double vtY = targetPos[1];
        double tof = 0;

        for (int i = 0; i < iterations; i++) {
            double dx = vtX - posX;
            double dy = vtY - posY;
            tof = tofFunc.applyAsDouble(Math.sqrt(dx * dx + dy * dy + dz2));
            vtX = targetPos[0] - velX * tof - 0.5 * accX * tof * tof;
            vtY = targetPos[1] - velY * tof - 0.5 * accY * tof * tof;
        }

        return new double[] { vtX, vtY };
    }
}
