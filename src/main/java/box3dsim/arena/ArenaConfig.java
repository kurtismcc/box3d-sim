package box3dsim.arena;

import box3dsim.world.WorldConfig;

/** Arena timing and world parameters. */
public record ArenaConfig(WorldConfig world, double physicsDt, int box3dSubSteps) {
    /** 4 ms physics steps (5 per 20 ms robot loop), 4 Box3D substeps each. */
    public static ArenaConfig defaults() {
        return new ArenaConfig(WorldConfig.defaults(), 0.004, 4);
    }
}
