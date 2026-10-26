package box3dsim.mechanism;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaParticipant;
import box3dsim.gamepiece.GamePieceType;
import box3dsim.motor.Battery;
import box3dsim.motor.ElectricalLoad;
import box3dsim.motor.LumpedMechanism;

/** Counted storage: hopper → indexer → one staged slot → feeder → launcher. */
public final class FuelStorage implements ArenaParticipant, ElectricalLoad, PieceSink {
    private final StorageConfig config;
    private final Launcher launcher;
    private final LumpedMechanism indexer;
    private final LumpedMechanism feeder;
    private final TokenBucket indexerBucket = new TokenBucket();
    private final TokenBucket feederBucket = new TokenBucket();
    private final double pitch;
    private int hopper;
    private boolean staged;

    public FuelStorage(Battery battery, StorageConfig config, Launcher launcher) {
        this.config = config;
        this.launcher = launcher;
        this.indexer = new LumpedMechanism(config.indexer().mechanism(), battery);
        this.feeder = new LumpedMechanism(config.feeder().mechanism(), battery);
        this.pitch = 2 * config.pieceType().radius() * 1.1;
        this.hopper = config.preload();
    }

    public void setIndexerInput(double volts, boolean coastAtZero) {
        indexer.setInput(volts, coastAtZero);
    }

    public void setFeederInput(double volts, boolean coastAtZero) {
        feeder.setInput(volts, coastAtZero);
    }

    public int hopperCount() {
        return hopper;
    }

    public boolean staged() {
        return staged;
    }

    public int totalCount() {
        return hopper + (staged ? 1 : 0);
    }

    public LumpedMechanism indexer() {
        return indexer;
    }

    public LumpedMechanism feeder() {
        return feeder;
    }

    @Override
    public boolean canAccept() {
        return totalCount() < config.capacity();
    }

    @Override
    public void accept(GamePieceType type) {
        hopper++;
    }

    @Override
    public void prePhysics(Arena arena, double dt) {
        indexer.step(dt);
        feeder.step(dt);
    }

    @Override
    public void postPhysics(Arena arena, double dt) {
        GamePieceType type = config.pieceType();
        RollerConfig ic = config.indexer();
        double indexSpeed = ic.surfaceSpeed(indexer);
        if (staged && indexSpeed <= -ic.minSurfaceSpeed()) {
            unstage();
        } else if (!staged && hopper > 0 && indexSpeed >= ic.minSurfaceSpeed()) {
            indexerBucket.refill(ic.efficiency() * indexSpeed / pitch, dt);
            if (indexerBucket.ready()) {
                indexerBucket.spend();
                hopper--;
                staged = true;
                indexer.removeEnergy(0.5 * type.mass() * indexSpeed * indexSpeed);
            }
        }

        RollerConfig fc = config.feeder();
        double feedSpeed = fc.surfaceSpeed(feeder);
        if (staged && feedSpeed <= -fc.minSurfaceSpeed()) {
            unstage();
        } else if (staged && feedSpeed >= fc.minSurfaceSpeed()) {
            feederBucket.refill(fc.efficiency() * feedSpeed / pitch, dt);
            if (feederBucket.ready() && launcher.launch(type)) {
                feederBucket.spend();
                staged = false;
                feeder.removeEnergy(0.5 * type.mass() * feedSpeed * feedSpeed);
            }
        }
    }

    private void unstage() {
        staged = false;
        hopper++;
    }

    @Override
    public void reset(Arena arena) {
        hopper = config.preload();
        staged = false;
        indexerBucket.reset();
        feederBucket.reset();
    }

    @Override
    public double supplyCurrent() {
        return indexer.supplyCurrent() + feeder.supplyCurrent();
    }
}
