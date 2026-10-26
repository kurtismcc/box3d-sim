package box3dsim.world;

/** Degrees of freedom a dynamic body may not move in. */
public record MotionLocks(
        boolean linearX, boolean linearY, boolean linearZ, boolean angularX, boolean angularY, boolean angularZ) {
    public static final MotionLocks NONE = new MotionLocks(false, false, false, false, false, false);
    /** Planar chassis: locks z translation, roll and pitch. */
    public static final MotionLocks PLANAR = new MotionLocks(false, false, true, true, true, false);
}
