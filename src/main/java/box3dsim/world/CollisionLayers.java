package box3dsim.world;

/** Collision layer bits and the masks they collide with. The table is symmetric. */
public final class CollisionLayers {
    public static final long FIELD = 1L << 0;
    public static final long ROBOT = 1L << 1;
    public static final long GAME_PIECE = 1L << 2;
    /** Invisible walls that stop robots but not game pieces. */
    public static final long ROBOT_BLOCKER = 1L << 3;
    /** Field geometry robots drive through but game pieces hit. */
    public static final long PIECE_BLOCKER = 1L << 4;
    public static final long SENSOR = 1L << 5;

    private CollisionLayers() {}

    public static long maskOf(long layerBit) {
        if (layerBit == FIELD) return ROBOT | GAME_PIECE;
        if (layerBit == ROBOT) return FIELD | ROBOT | GAME_PIECE | ROBOT_BLOCKER;
        if (layerBit == GAME_PIECE) return FIELD | ROBOT | GAME_PIECE | PIECE_BLOCKER | SENSOR;
        if (layerBit == ROBOT_BLOCKER) return ROBOT;
        if (layerBit == PIECE_BLOCKER) return GAME_PIECE;
        if (layerBit == SENSOR) return GAME_PIECE;
        throw new IllegalArgumentException("Unknown collision layer bit " + layerBit);
    }

    public static CollisionFilter of(long layerBit) {
        return new CollisionFilter(layerBit, maskOf(layerBit));
    }
}
