package box3dsim.mechanism;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaParticipant;
import box3dsim.gamepiece.GamePiece;
import box3dsim.gamepiece.GamePieceType;
import box3dsim.math.Pose3;
import box3dsim.math.Quat;
import box3dsim.math.Vec3;
import box3dsim.motor.Battery;
import box3dsim.motor.ElectricalLoad;
import box3dsim.motor.LumpedMechanism;
import box3dsim.world.BodyHandle;
import box3dsim.world.CollisionLayers;
import box3dsim.world.ShapeHandle;
import box3dsim.world.ShapeOptions;
import box3dsim.world.ShapeSpec;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Ground intake: takes pieces from a sensor zone on the chassis while deployed and spinning inward. */
public final class IntakeSim implements ArenaParticipant, ElectricalLoad {
    private final Arena arena;
    private final BodyHandle chassis;
    private final IntakeConfig config;
    private final PieceSink sink;
    private final LumpedMechanism roller;
    private final LumpedMechanism pivot;
    private final ShapeHandle zone;
    private final TokenBucket bucket = new TokenBucket();
    private final double pitch;
    private int acquired;

    public IntakeSim(Arena arena, BodyHandle chassis, Battery battery, IntakeConfig config, PieceSink sink) {
        this.arena = arena;
        this.chassis = chassis;
        this.config = config;
        this.sink = sink;
        this.roller = new LumpedMechanism(config.roller().mechanism(), battery);
        this.pivot = new LumpedMechanism(config.pivot(), battery);
        this.pivot.setState(config.stowedPosition(), 0);
        this.zone = arena.world().addShape(chassis,
                new ShapeSpec.Box(config.zoneHalfExtents(), new Pose3(config.zoneCenter(), Quat.IDENTITY)),
                ShapeOptions.sensor(CollisionLayers.SENSOR));
        this.pitch = 2 * config.pieceType().radius() * 1.1;
    }

    public void setRollerInput(double volts, boolean coastAtZero) {
        roller.setInput(volts, coastAtZero);
    }

    public void setPivotInput(double volts, boolean coastAtZero) {
        pivot.setInput(volts, coastAtZero);
    }

    public LumpedMechanism roller() {
        return roller;
    }

    public LumpedMechanism pivot() {
        return pivot;
    }

    /** 0 = stowed, 1 = fully deployed. */
    public double deployProgress() {
        double span = config.deployedPosition() - config.stowedPosition();
        return Math.clamp((pivot.position() - config.stowedPosition()) / span, 0, 1);
    }

    /** Pieces taken since the last reset. */
    public int acquiredCount() {
        return acquired;
    }

    @Override
    public void prePhysics(Arena arena, double dt) {
        roller.step(dt);
        pivot.step(dt);
    }

    @Override
    public void postPhysics(Arena arena, double dt) {
        double surface = config.roller().surfaceSpeed(roller);
        bucket.refill(config.roller().efficiency() * surface / pitch, dt);
        if (deployProgress() < config.deployedFraction()
                || surface < config.roller().minSurfaceSpeed()
                || !bucket.ready()
                || !sink.canAccept()) {
            return;
        }
        List<GamePiece> candidates = new ArrayList<>();
        for (ShapeHandle s : arena.world().sensorOverlaps(zone)) {
            arena.gamePieces().byShape(s)
                    .filter(p -> p.type().equals(config.pieceType()))
                    .ifPresent(candidates::add);
        }
        if (candidates.isEmpty()) {
            return;
        }
        Vec3 center = arena.world().pose(chassis).transformPoint(config.zoneCenter());
        candidates.sort(Comparator.comparingDouble(p -> p.pose().position().minus(center).normSquared()));
        GamePieceType type = config.pieceType();
        for (GamePiece p : candidates) {
            if (!bucket.ready() || !sink.canAccept()) {
                break;
            }
            arena.gamePieces().remove(p);
            sink.accept(type);
            bucket.spend();
            roller.removeEnergy(0.5 * type.mass() * surface * surface);
            acquired++;
        }
    }

    @Override
    public void reset(Arena arena) {
        acquired = 0;
        bucket.reset();
    }

    @Override
    public double supplyCurrent() {
        return roller.supplyCurrent() + pivot.supplyCurrent();
    }
}
