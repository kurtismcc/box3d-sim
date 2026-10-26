package box3dsim.gamepiece;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaParticipant;
import box3dsim.math.Pose3;
import box3dsim.math.Quat;
import box3dsim.math.Vec3;
import box3dsim.world.BodyHandle;
import box3dsim.world.BodyMove;
import box3dsim.world.BodySpec;
import box3dsim.world.CollisionLayers;
import box3dsim.world.Material;
import box3dsim.world.ShapeHandle;
import box3dsim.world.ShapeOptions;
import box3dsim.world.ShapeSpec;
import box3dsim.world.SimWorld;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/** Game pieces as rigid bodies, with air drag, Magnus lift and out-of-field removal. */
public final class GamePieces implements ArenaParticipant {
    private static final double AIR_DENSITY = 1.204;
    private static final double MIN_AERO_SPEED = 0.5;
    private static final double BULLET_SPEED = 5.0;
    private static final double OUT_OF_FIELD_MARGIN = 1.0;
    private static final double MIN_Z = -0.5;
    private static final double GRAVITY = 9.81;
    private static final double ON_FLOOR_TOLERANCE = 0.01;

    private final SimWorld world;
    private final Map<Integer, GamePiece> live = new LinkedHashMap<>();
    private final Map<Long, GamePiece> byBody = new HashMap<>();
    private final Map<Long, GamePiece> byShape = new HashMap<>();
    private final Set<GamePiece> awake = new LinkedHashSet<>();
    private int nextId = 1;
    private boolean aerodynamics = true;
    private double minX = Double.NEGATIVE_INFINITY;
    private double maxX = Double.POSITIVE_INFINITY;
    private double minY = Double.NEGATIVE_INFINITY;
    private double maxY = Double.POSITIVE_INFINITY;
    private int outOfField;

    public GamePieces(SimWorld world) {
        this.world = world;
    }

    public GamePiece spawn(GamePieceType type, Vec3 position, Vec3 velocity, Vec3 angularVelocity, boolean awakeNow) {
        Pose3 pose = new Pose3(position, Quat.IDENTITY);
        BodyHandle body = world.createBody(BodySpec.dynamic(pose)
                .withVelocity(velocity, angularVelocity)
                .withDamping(type.linearDamping(), type.angularDamping())
                .withAwake(awakeNow)
                .withBullet(velocity.norm() > BULLET_SPEED));
        ShapeHandle shape = world.addShape(body, new ShapeSpec.Sphere(Vec3.ZERO, type.radius()),
                ShapeOptions.solid(withoutRollingResistance(type.material()), type.density(),
                        CollisionLayers.GAME_PIECE));
        GamePiece piece = new GamePiece(nextId++, type, body, shape, pose, awakeNow);
        live.put(piece.id(), piece);
        byBody.put(body.id(), piece);
        byShape.put(shape.id(), piece);
        if (awakeNow) {
            awake.add(piece);
        }
        return piece;
    }

    /** Destroys the body; the piece becomes dead. */
    public void remove(GamePiece piece) {
        if (!piece.alive) {
            return;
        }
        piece.alive = false;
        live.remove(piece.id());
        byBody.remove(piece.body().id());
        byShape.remove(piece.shape().id());
        awake.remove(piece);
        world.destroyBody(piece.body());
    }

    public Optional<GamePiece> byShape(ShapeHandle shape) {
        return Optional.ofNullable(byShape.get(shape.id()));
    }

    /** Live pieces, unmodifiable view. */
    public Collection<GamePiece> all() {
        return Collections.unmodifiableCollection(live.values());
    }

    public List<Pose3> poses(GamePieceType type) {
        List<Pose3> out = new ArrayList<>();
        for (GamePiece p : live.values()) {
            if (p.type().equals(type)) {
                out.add(p.pose);
            }
        }
        return out;
    }

    public int count(GamePieceType type) {
        int n = 0;
        for (GamePiece p : live.values()) {
            if (p.type().equals(type)) {
                n++;
            }
        }
        return n;
    }

    public int awakeCount() {
        return awake.size();
    }

    public void clear() {
        for (GamePiece p : new ArrayList<>(live.values())) {
            remove(p);
        }
    }

    public double airDensity() {
        return AIR_DENSITY;
    }

    /** On by default. */
    public void setAerodynamicsEnabled(boolean on) {
        aerodynamics = on;
    }

    /** Default unbounded; pieces more than 1 m outside these bounds are removed. */
    public void setFieldBounds(double minX, double maxX, double minY, double maxY) {
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    /** Pieces removed for leaving the field since the last reset. */
    public int outOfFieldCount() {
        return outOfField;
    }

    /** Updates cached poses and the awake set from the world's body move events. */
    public void syncFromWorld() {
        for (BodyMove move : world.bodyMoves()) {
            GamePiece p = byBody.get(move.body().id());
            if (p == null) {
                continue;
            }
            p.pose = move.pose();
            p.awake = !move.fellAsleep();
            if (p.awake) {
                awake.add(p);
            } else {
                awake.remove(p);
            }
        }
    }

    /**
     * Box3D's own rolling resistance stops a sliding sphere from ever spinning up, so pushed or launched pieces
     * would skid to a halt. Shapes get zero rolling resistance and prePhysics applies it as a force instead.
     */
    private static Material withoutRollingResistance(Material m) {
        return new Material(m.friction(), m.restitution(), 0);
    }

    @Override
    public void prePhysics(Arena arena, double dt) {
        for (GamePiece p : awake) {
            GamePieceType t = p.type();
            boolean onFloor = p.pose.position().z() < t.radius() + ON_FLOOR_TOLERANCE;
            boolean rolling = onFloor && t.material().rollingResistance() > 0;
            if (!rolling && !aerodynamics) {
                continue;
            }
            Vec3 v = world.linearVelocity(p.body());
            if (rolling && applyRollingResistance(p, v, dt)) {
                continue;
            }
            double speed = v.norm();
            if (!aerodynamics || speed < MIN_AERO_SPEED) {
                continue;
            }
            double area = Math.PI * t.radius() * t.radius();
            Vec3 w = world.angularVelocity(p.body());
            Vec3 force = v.times(-0.5 * AIR_DENSITY * t.dragCoefficient() * area * speed);
            Vec3 wxv = w.cross(v);
            double wxvNorm = wxv.norm();
            if (wxvNorm > 1e-9) {
                double spinRatio = t.radius() * w.norm() / speed;
                double lift = Math.min(t.liftMax(), t.liftSlope() * spinRatio);
                force = force.plus(wxv.times(0.5 * AIR_DENSITY * area * lift * speed * speed / wxvNorm));
            }
            world.applyForceToCenter(p.body(), force);
            if (p.pose.position().z() > t.radius() + 0.01 && t.spinDecayPerSecond() > 0) {
                world.setVelocity(p.body(), v, w.times(Math.exp(-t.spinDecayPerSecond() * dt)));
            }
        }
    }

    /** Applies F = −C_rr·m·g along the ground velocity; returns true if the piece was stopped instead. */
    private boolean applyRollingResistance(GamePiece p, Vec3 v, double dt) {
        GamePieceType t = p.type();
        double decel = t.material().rollingResistance() * GRAVITY;
        double groundSpeed = Math.hypot(v.x(), v.y());
        if (groundSpeed <= decel * dt) {
            if (groundSpeed > 0 && Math.abs(v.z()) < 0.05) {
                world.setVelocity(p.body(), new Vec3(0, 0, v.z()), Vec3.ZERO);
                return true;
            }
            return false;
        }
        double scale = -t.mass() * decel / groundSpeed;
        world.applyForceToCenter(p.body(), new Vec3(v.x() * scale, v.y() * scale, 0));
        return false;
    }

    @Override
    public void postPhysics(Arena arena, double dt) {
        List<GamePiece> gone = new ArrayList<>();
        for (GamePiece p : awake) {
            Vec3 pos = p.pose.position();
            if (pos.z() < MIN_Z
                    || pos.x() < minX - OUT_OF_FIELD_MARGIN
                    || pos.x() > maxX + OUT_OF_FIELD_MARGIN
                    || pos.y() < minY - OUT_OF_FIELD_MARGIN
                    || pos.y() > maxY + OUT_OF_FIELD_MARGIN) {
                gone.add(p);
            }
        }
        for (GamePiece p : gone) {
            remove(p);
            outOfField++;
        }
    }

    /** Resets the out-of-field count only; seasons clear and respawn pieces themselves. */
    @Override
    public void reset(Arena arena) {
        outOfField = 0;
    }
}
