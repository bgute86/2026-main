package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

/**
 * Kalman Filter + Newton-Raphson Solver with rotational velocity support.
 *
 * Extends KalmanNewtonSOTM by adding omega and shooter offset to account
 * for tangential velocity at an off-center shooter during rotation.
 *
 * Usage:
 *     RotatingKalmanNewtonSOTM solver = new RotatingKalmanNewtonSOTM(50.0, 12.0, 2);
 *
 *     // Runtime tuning (e.g. from SmartDashboard):
 *     solver.setQ(60.0);
 *     solver.setApproxProjectileSpeed(13.0);
 *     solver.setSensitivity(0.8, 0.8, 1.0);  // velScale, accelScale, omegaScale
 *
 *     // Every loop cycle:
 *     solver.predict(dt);
 *     solver.updateVelocity(vx, vy, 0.03);
 *     solver.updateAcceleration(ax, ay, 0.10);
 *
 *     // When vision arrives:
 *     solver.updatePosition(x, y, 0.05);
 *
 *     // When shooting:
 *     double[] vt = solver.solve(targetPos, tofFunc, 0.08, deltaZ, omega, shooterOffset);
 *
 *     // After robot disable/enable:
 *     solver.reset();
 */
public class RotatingKalmanNewtonSOTM {

    private final AxisKF kfX;
    private final AxisKF kfY;
    private double approxProjectileSpeed;
    private int newtonIterations;
    private double[] sensitivity = {1.0, 1.0, 1.0}; // {velScale, accelScale, omegaScale}

    /**
     * @param q                      jerk spectral density (m^2/s^5). Start at 50.
     * @param approxProjectileSpeed  rough horizontal ball speed (m/s) for initial guess.
     * @param newtonIterations       Newton-Raphson steps (2 is enough).
     */
    public RotatingKalmanNewtonSOTM(double q, double approxProjectileSpeed, int newtonIterations) {
        this.kfX = new AxisKF(q);
        this.kfY = new AxisKF(q);
        this.approxProjectileSpeed = approxProjectileSpeed;
        this.newtonIterations = newtonIterations;
    }

    public RotatingKalmanNewtonSOTM() {
        this(50.0, 12.0, 2);
    }

    // ---- Tuning interface ----

    /**
     * Set jerk spectral density. Higher = more responsive, noisier.
     * Takes effect on the next predict() call.
     */
    public void setQ(double q) {
        kfX.setQ(q);
        kfY.setQ(q);
    }

    public double getQ() { return kfX.q; }

    public void setApproxProjectileSpeed(double speed) { this.approxProjectileSpeed = speed; }
    public double getApproxProjectileSpeed() { return approxProjectileSpeed; }

    public void setNewtonIterations(int n) { this.newtonIterations = n; }
    public int getNewtonIterations() { return newtonIterations; }

    /**
     * Scale how much each correction component is applied.
     * 1.0 = full correction, 0.0 = ignore, values between = partial.
     *
     * @param velScale    velocity lead scaling
     * @param accelScale  acceleration lead scaling
     * @param omegaScale  rotational tangential velocity scaling
     */
    public void setSensitivity(double velScale, double accelScale, double omegaScale) {
        this.sensitivity = new double[] { velScale, accelScale, omegaScale };
    }

    public double[] getSensitivity() { return sensitivity.clone(); }

    /**
     * Reset Kalman filter state and covariance to initial values.
     * Call after robot disable/enable or when state is known to be stale.
     */
    public void reset() {
        kfX.reset();
        kfY.reset();
    }

    // ---- Kalman filter interface ----

    public void predict(double dt) {
        kfX.predict(dt);
        kfY.predict(dt);
    }

    public void updateVelocity(double[] vel, double sigma) {
        double R = sigma * sigma;
        kfX.updateVelocity(vel[0], R);
        kfY.updateVelocity(vel[1], R);
    }

    public void updateAcceleration(double[] accel, double sigma) {
        double R = sigma * sigma;
        kfX.updateAcceleration(accel[0], R);
        kfY.updateAcceleration(accel[1], R);
    }

    public void updatePosition(double[] pos, double sigma) {
        double R = sigma * sigma;
        kfX.updatePosition(pos[0], R);
        kfY.updatePosition(pos[1], R);
    }

    // ---- State access ----

    public double[] getPosition()     { return new double[] { kfX.x[0], kfY.x[0] }; }
    public double[] getVelocity()     { return new double[] { kfX.x[1], kfY.x[1] }; }
    public double[] getAcceleration() { return new double[] { kfX.x[2], kfY.x[2] }; }

    // ---- Solver ----

    /**
     * @param targetPos      {x, y} field-relative target position (m)
     * @param tofFunc        distance (m) -> time-of-flight (s)
     * @param latency        total system latency to compensate (s)
     * @param deltaZ         target z - robot z (m), constant
     * @param omega          robot angular velocity (rad/s), positive = CCW
     * @return {x, y} virtual target position
     */
    public double[] solve(double[] targetPos, DoubleUnaryOperator tofFunc,
                          double latency, double deltaZ, double omega) {
        return solve(targetPos, tofFunc, latency, deltaZ, omega, new double[] {0, 0});
    }

    public double[] solve(double[] targetPos, DoubleUnaryOperator tofFunc,
                          double latency, double deltaZ, double omega,
                          double[] shooterOffset) {

        double[] predX = kfX.predictForward(latency);
        double[] predY = kfY.predictForward(latency);

        double velScale   = sensitivity[0];
        double accelScale = sensitivity[1];
        double omegaScale = sensitivity[2];

        double vX = predX[1] * velScale, aX = predX[2] * accelScale;
        double vY = predY[1] * velScale, aY = predY[2] * accelScale;

        // add tangential vel from rotation at shooter offset
        if (shooterOffset != null && omega != 0) {
            vX += -omega * shooterOffset[1] * omegaScale;
            vY +=  omega * shooterOffset[0] * omegaScale;
        }

        double pX = predX[0];
        double pY = predY[0];
        double dz2 = deltaZ * deltaZ;

        double dx = targetPos[0] - pX;
        double dy = targetPos[1] - pY;
        double d2 = dx * dx + dy * dy;

        if (d2 < 1e-12) return new double[] { targetPos[0], targetPos[1] };

        // closed-form initial guess
        double s = approxProjectileSpeed;
        double s2 = s * s;
        double v2 = vX * vX + vY * vY;
        double dDotA = dx * aX + dy * aY;
        double dDotV = dx * vX + dy * vY;

        double A = s2 - v2 + dDotA;
        double B = dDotV;
        double disc = B * B + A * d2;

        double fallback = (s > 0) ? Math.sqrt(d2) / s : 0.1;
        double tau;
        if (disc < 0 || A == 0) {
            tau = fallback;
        } else {
            tau = (-B + Math.sqrt(disc)) / A;
            if (tau < 0) tau = fallback;
        }
        if (!Double.isFinite(tau) || tau <= 0) tau = fallback;

        // NR refinement
        double haX = aX * 0.5, haY = aY * 0.5;
        double eps = 0.001;

        for (int n = 0; n < newtonIterations; n++) {
            double px = dx - vX * tau - haX * tau * tau;
            double py = dy - vY * tau - haY * tau * tau;
            double rv = Math.sqrt(px * px + py * py + dz2);
            if (rv < 1e-6) break;

            double tofVal = tofFunc.applyAsDouble(rv);
            if (!Double.isFinite(tofVal) || tofVal <= 0) break;
            double h = tau - tofVal;

            double dpdtX = -(vX + aX * tau);
            double dpdtY = -(vY + aY * tau);
            double drvdt = (px * dpdtX + py * dpdtY) / rv;
            double rvPlus = Math.max(rv + eps, eps);
            double rvMinus = Math.max(rv - eps, eps);
            double fPrime = (tofFunc.applyAsDouble(rvPlus)
                           - tofFunc.applyAsDouble(rvMinus)) / (rvPlus - rvMinus);
            double hPrime = 1.0 - fPrime * drvdt;

            if (Math.abs(hPrime) < 1e-10) break;

            tau -= h / hPrime;
            tau = Math.max(tau, 0.01);
        }

        if (!Double.isFinite(tau) || tau <= 0) return new double[] { targetPos[0], targetPos[1] };

        return new double[] {
            targetPos[0] - vX * tau - haX * tau * tau,
            targetPos[1] - vY * tau - haY * tau * tau
        };
    }

    /**
     * Probability the ball lands within targetRadius of aim point.
     *
     * @param targetRadius  effective target radius (m), e.g. 0.30
     * @param tof           time-of-flight (s)
     * @return P(hit) in [0, 1]
     */
    public double hitProbability(double targetRadius, double tof) {
        double sx2 = kfX.predictPositionVariance(tof);
        double sy2 = kfY.predictPositionVariance(tof);
        double sigma2 = sx2 + sy2;
        if (sigma2 <= 0 || !Double.isFinite(sigma2)) return 0.0;
        return 1.0 - Math.exp(-targetRadius * targetRadius / (2.0 * sigma2));
    }

    // ========================================================================
    // Internal: per-axis 3-state Kalman filter [position, velocity, acceleration]
    // ========================================================================

    private static class AxisKF {
        double[] x = new double[3];
        double[][] P = { {1,0,0}, {0,1,0}, {0,0,10} };
        double q;

        AxisKF(double q) { this.q = q; }

        void setQ(double q) { this.q = q; }

        void reset() {
            x = new double[3];
            P = new double[][] { {1,0,0}, {0,1,0}, {0,0,10} };
        }

        void predict(double dt) {
            double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt, dt5 = dt4*dt;

            double newP = x[0] + x[1]*dt + 0.5*x[2]*dt2;
            double newV = x[1] + x[2]*dt;
            x[0] = newP; x[1] = newV;

            double[][] F = { {1,dt,0.5*dt2}, {0,1,dt}, {0,0,1} };
            double[][] Q = {
                {q*dt5/20, q*dt4/8, q*dt3/6},
                {q*dt4/8,  q*dt3/3, q*dt2/2},
                {q*dt3/6,  q*dt2/2, q*dt}
            };

            double[][] FP = matMul(F, P);
            double[][] FPFt = matMul(FP, transpose(F));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    P[i][j] = FPFt[i][j] + Q[i][j];
        }

        void updateVelocity(double z, double R)     { scalarUpdate(new double[]{0,1,0}, z, R); }
        void updateAcceleration(double z, double R)  { scalarUpdate(new double[]{0,0,1}, z, R); }
        void updatePosition(double z, double R)      { scalarUpdate(new double[]{1,0,0}, z, R); }

        double[] predictForward(double dt) {
            return new double[] {
                x[0] + x[1]*dt + 0.5*x[2]*dt*dt,
                x[1] + x[2]*dt,
                x[2]
            };
        }

        double predictPositionVariance(double dt) {
            double dt2 = dt*dt, dt3 = dt*dt*dt, dt4 = dt3*dt, dt5 = dt4*dt;
            double[][] F = { {1,dt,0.5*dt2}, {0,1,dt}, {0,0,1} };
            double[][] Q = {
                {q*dt5/20, q*dt4/8, q*dt3/6},
                {q*dt4/8,  q*dt3/3, q*dt2/2},
                {q*dt3/6,  q*dt2/2, q*dt}
            };
            double[][] FPFt = matMul(matMul(F, P), transpose(F));
            return FPFt[0][0] + Q[0][0];
        }

        private void scalarUpdate(double[] H, double z, double R) {
            double Hx = H[0]*x[0] + H[1]*x[1] + H[2]*x[2];
            double y = z - Hx;

            double S = R;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    S += H[i] * P[i][j] * H[j];

            double[] K = new double[3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++)
                    K[i] += P[i][j] * H[j];
                K[i] /= S;
            }

            for (int i = 0; i < 3; i++)
                x[i] += K[i] * y;

            double[][] Pnew = new double[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    Pnew[i][j] = P[i][j];
                    for (int k = 0; k < 3; k++)
                        Pnew[i][j] -= K[i] * H[k] * P[k][j];
                }
            P = Pnew;
        }

        private static double[][] matMul(double[][] A, double[][] B) {
            double[][] C = new double[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    for (int k = 0; k < 3; k++)
                        C[i][j] += A[i][k] * B[k][j];
            return C;
        }

        private static double[][] transpose(double[][] A) {
            double[][] T = new double[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    T[i][j] = A[j][i];
            return T;
        }
    }
}
