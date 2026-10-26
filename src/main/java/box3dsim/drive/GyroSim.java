package box3dsim.drive;

import java.util.Random;

/** Yaw gyro that accumulates the body's actual yaw change plus configured errors. */
public final class GyroSim {
    private final GyroConfig config;
    private final Random random;
    private double yaw;
    private double rate;
    private double prevBodyYaw;
    private double prevTrueRate;

    public GyroSim(GyroConfig config, double initialBodyYaw) {
        this.config = config;
        this.random = new Random(config.seed());
        this.prevBodyYaw = initialBodyYaw;
    }

    void update(double bodyYaw, double dt) {
        double trueRate = wrapToPi(bodyYaw - prevBodyYaw) / dt;
        prevBodyYaw = bodyYaw;

        rate = trueRate + random.nextGaussian() * config.velocityNoiseFraction() * Math.abs(trueRate);
        yaw += rate * dt;

        double accel = (trueRate - prevTrueRate) / dt;
        prevTrueRate = trueRate;
        if (Math.abs(accel) > config.impactAccelThreshold()) {
            yaw -= Math.signum(accel) * Math.abs(accel) / config.impactAccelThreshold()
                    * config.impactDriftRadPerMultiple();
        }

        double driftStdDeg = config.driftDegPer30sMotionless() / 30 * dt;
        if (driftStdDeg > 0) {
            yaw += Math.toRadians(random.nextGaussian() * driftStdDeg);
        }
    }

    /** Re-anchors to a teleported body without changing the reading. */
    void rebase(double bodyYaw) {
        prevBodyYaw = bodyYaw;
        prevTrueRate = 0;
    }

    /** Continuous yaw reading, rad. */
    public double yawRad() {
        return yaw;
    }

    /** Measured yaw rate, rad/s. */
    public double yawRateRadPerSec() {
        return rate;
    }

    public void setYaw(double rad) {
        yaw = rad;
    }

    static double wrapToPi(double a) {
        return Math.IEEEremainder(a, 2 * Math.PI);
    }
}
