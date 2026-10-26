package box3dsim.gamepiece;

import box3dsim.math.Pose3;
import box3dsim.world.BodyHandle;
import box3dsim.world.ShapeHandle;

/** A live game piece body. Dead once removed. */
public final class GamePiece {
    private final int id;
    private final GamePieceType type;
    private final BodyHandle body;
    private final ShapeHandle shape;
    Pose3 pose;
    boolean awake;
    boolean alive = true;

    GamePiece(int id, GamePieceType type, BodyHandle body, ShapeHandle shape, Pose3 pose, boolean awake) {
        this.id = id;
        this.type = type;
        this.body = body;
        this.shape = shape;
        this.pose = pose;
        this.awake = awake;
    }

    /** Unique per Arena and never reused. */
    public int id() {
        return id;
    }

    public GamePieceType type() {
        return type;
    }

    /** Cached pose, updated from body move events. */
    public Pose3 pose() {
        return pose;
    }

    public boolean isAwake() {
        return awake;
    }

    public boolean isAlive() {
        return alive;
    }

    BodyHandle body() {
        return body;
    }

    ShapeHandle shape() {
        return shape;
    }
}
