package frc.robot.subsystems;


import java.util.function.DoubleUnaryOperator;

/**
 * Method 2: Team 254 Radial-Tangential Decomposition
 *
 * Decomposes velocity into radial (toward/away) and tangential (orbiting).
 * Radial velocity adjusts effective distance; tangential velocity drives
 * turret lead angle. Iterates to converge TOF and lead.
 */
public class RadialTangential254 {

    private final int iterations;

    public RadialTangential254() { this(5); }

    public RadialTangential254(int iterations) {
        this.iterations = iterations;
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

        double dx = targetPos[0] - posX;
        double dy = targetPos[1] - posY;
        double dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 1e-6) return new double[] { targetPos[0], targetPos[1] };

        double dhX = dx / dist;
        double dhY = dy / dist;

        double vRad = robotVel[0] * dhX + robotVel[1] * dhY;
        double vTanX = robotVel[0] - dhX * vRad;
        double vTanY = robotVel[1] - dhY * vRad;
        double vTanMag = Math.sqrt(vTanX * vTanX + vTanY * vTanY);

        double cross = Math.signum(dx * robotVel[1] - dy * robotVel[0]);
        double baseAngle = Math.atan2(dy, dx);

        double effDist = dist;
        double tof = 0;
        double lead = 0;

        for (int i = 0; i < iterations; i++) {
            tof = tofFunc.applyAsDouble(effDist);
            if (!Double.isFinite(tof) || tof <= 0) break;
            effDist = dist - vRad * tof;
            effDist = Math.max(effDist, 0.1);
            lead = Math.atan2(vTanMag * tof, effDist);
        }

        if (!Double.isFinite(tof) || tof <= 0) return new double[] { targetPos[0], targetPos[1] };

        lead *= -cross;
        double aim = baseAngle + lead;

        double vtX = posX + Math.cos(aim) * effDist;
        double vtY = posY + Math.sin(aim) * effDist;

        return new double[] { vtX, vtY };
    }
}
