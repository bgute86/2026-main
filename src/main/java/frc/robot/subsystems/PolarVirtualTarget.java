package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

/**
 * Virtual Target using polar (radial/tangential) decomposition with
 * 2nd-order (acceleration) correction.
 *
 * Decomposes robot velocity and acceleration into radial (toward/away from
 * target) and tangential (orbiting) components. The radial component adjusts
 * effective distance; the tangential component drives the angular lead.
 *
 * Key advantage over the Cartesian approach (MoreComplicatedVirtualTarget):
 *   - Purely radial motion produces a virtual target on the same line
 *     (no angular deviation).
 *   - The radial and tangential corrections are computed independently,
 *     preventing the self-reinforcing inflation seen in Cartesian iteration.
 *
 * offset_radial     = -vRad * tof - 0.5 * aRad * tof^2
 * offset_tangential = -vTan * tof - 0.5 * aTan * tof^2
 */
public class PolarVirtualTarget {

    private final int iterations;
    // sensitivity scalars: {radial, tangential, omega}. 1.0 = full correction, 0.0 = no correction
    private double[] sensitivity = {1.0, 1.0, 1.0};

    public PolarVirtualTarget(int iterations) {
        this.iterations = iterations;
    }

    public PolarVirtualTarget() {
        this(10);
    }

    /** Set sensitivity for {radial, tangential, omega}. 1.0 = default, &lt;1 = less correction. */
    public void setSensitivity(double radialScale, double tangentialScale, double omegaScale) {
        this.sensitivity = new double[] {radialScale, tangentialScale, omegaScale};
    }

    /**
     * @param robotPos       {x, y} field-relative position (m)
     * @param robotVel       {vx, vy} field-relative velocity (m/s)
     * @param robotAcc       {ax, ay} field-relative acceleration (m/s^2)
     * @param targetPos      {x, y} field-relative target position (m)
     * @param tofFunc        distance (m) -&gt; time-of-flight (s)
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

        // apply omega sensitivity scaling
        double scaledOmega = omega * sensitivity[2];
        double velX = robotVel[0];
        double velY = robotVel[1];
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

        // --- Polar decomposition ---
        double realDx = targetPos[0] - posX;
        double realDy = targetPos[1] - posY;
        double realDist = Math.sqrt(realDx * realDx + realDy * realDy);
        if (realDist < 1e-9) {
            return new double[] { targetPos[0], targetPos[1] };
        }

        // Unit vectors: radial (toward target) and tangential (90 deg CCW)
        double rHatX = realDx / realDist;
        double rHatY = realDy / realDist;
        double tHatX = -rHatY;
        double tHatY =  rHatX;

        // Decompose velocity and acceleration, then apply sensitivity
        double vRad = (velX * rHatX + velY * rHatY) * sensitivity[0];
        double vTan = (velX * tHatX + velY * tHatY) * sensitivity[1];
        double aRad = (accX * rHatX + accY * rHatY) * sensitivity[0];
        double aTan = (accX * tHatX + accY * tHatY) * sensitivity[1];

        // Initial TOF from real 3D distance
        double tof = tofFunc.applyAsDouble(Math.sqrt(realDist * realDist + dz2));
        if (!Double.isFinite(tof) || tof <= 0) {
            return new double[] { targetPos[0], targetPos[1] };
        }

        // Iterate TOF to convergence.
        // Both radial and tangential drift change the effective distance
        // the ball must cross, so both feed into the TOF calculation.
        // Critically, BOTH components use the SAME converged tof.
        double radialDrift = 0;
        double tangentialDrift = 0;

        for (int i = 0; i < iterations; i++) {
            radialDrift     = vRad * tof + 0.5 * aRad * tof * tof;
            tangentialDrift = vTan * tof + 0.5 * aTan * tof * tof;

            // Effective radial distance (clamped to prevent sign flip)
            double effRadial = realDist - radialDrift;
            if (effRadial < 0.05) effRadial = 0.05;

            // 3D effective distance for TOF lookup
            double effDist3D = Math.sqrt(effRadial * effRadial
                                       + tangentialDrift * tangentialDrift
                                       + dz2);
            tof = tofFunc.applyAsDouble(effDist3D);
            if (!Double.isFinite(tof) || tof <= 0) {
                return new double[] { targetPos[0], targetPos[1] };
            }
        }

        // --- Reconstruct virtual target in Cartesian ---
        // Standard VT: offset the target by the robot's predicted displacement
        // during time-of-flight, decomposed into radial and tangential.
        //
        // rEff moves the aim point closer/farther along the radial direction.
        // This is correct: when the robot approaches the target, the shorter
        // aim distance causes shootAt() to use a lower launch speed, but the
        // ball inherits the robot's radial velocity, so it still reaches the
        // real target at the correct time with the correct vertical arc.
        double rEff = realDist - radialDrift;
        if (rEff < 0.05) rEff = 0.05;

        double vtX = posX + rEff * rHatX - tangentialDrift * tHatX;
        double vtY = posY + rEff * rHatY - tangentialDrift * tHatY;

        return new double[] { vtX, vtY };
    }
}