package box3dsim.drive;

/** Gyro error model; noise comes from a seeded Random so runs are repeatable. */
public record GyroConfig(
        double driftDegPer30sMotionless,
        double velocityNoiseFraction,
        double impactAccelThreshold,
        double impactDriftRadPerMultiple,
        long seed) {

    public static GyroConfig ideal() {
        return new GyroConfig(0, 0, Double.POSITIVE_INFINITY, 0, 0);
    }

    /** maple-sim's Pigeon 2 values. */
    public static GyroConfig pigeon2() {
        return new GyroConfig(0.5, 0.02, 500, Math.toRadians(1), 4470);
    }
}
