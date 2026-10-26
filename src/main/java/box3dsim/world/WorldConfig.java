package box3dsim.world;

import box3dsim.math.Vec3;

/** World creation parameters. */
public record WorldConfig(
        Vec3 gravity,
        int workerCount,
        boolean enableSleep,
        boolean enableContinuous,
        double restitutionThreshold,
        double maximumLinearSpeed) {

    /** z-up gravity, one worker (deterministic), sleep and continuous on, 0.5 m/s bounce threshold, 30 m/s cap. */
    public static WorldConfig defaults() {
        return new WorldConfig(new Vec3(0, 0, -9.81), 1, true, true, 0.5, 30.0);
    }
}
