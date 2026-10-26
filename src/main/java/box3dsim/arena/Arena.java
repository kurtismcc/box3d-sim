package box3dsim.arena;

import box3dsim.gamepiece.GamePieces;
import box3dsim.motor.Battery;
import box3dsim.motor.ElectricalLoad;
import box3dsim.world.Box3dWorld;
import box3dsim.world.SimWorld;
import java.util.ArrayList;
import java.util.List;

/**
 * Owns the world, battery and participants and steps them in a fixed order: participants' prePhysics, world step,
 * game piece pose sync, participants' postPhysics, battery update.
 */
public final class Arena implements AutoCloseable {
    private final ArenaConfig config;
    private final SimWorld world;
    private final Battery battery = Battery.defaults();
    private final GamePieces gamePieces;
    private final List<ArenaParticipant> participants = new ArrayList<>();
    private final List<ElectricalLoad> loads = new ArrayList<>();
    private double time;
    private ArenaStats lastStats = new ArenaStats(0, 0, 0);

    public Arena(ArenaConfig config) {
        this.config = config;
        this.world = new Box3dWorld(config.world());
        this.gamePieces = new GamePieces(world);
        participants.add(gamePieces);
    }

    public SimWorld world() {
        return world;
    }

    public Battery battery() {
        return battery;
    }

    public GamePieces gamePieces() {
        return gamePieces;
    }

    public ArenaConfig config() {
        return config;
    }

    /** Simulated seconds since construction. */
    public double time() {
        return time;
    }

    /** Stepped in insertion order, always before the game pieces' own aerodynamics participant. */
    public void addParticipant(ArenaParticipant participant) {
        participants.add(participants.size() - 1, participant);
    }

    public void addLoad(ElectricalLoad load) {
        loads.add(load);
    }

    /** One physics step of config.physicsDt. */
    public void step() {
        long t0 = System.nanoTime();
        double dt = config.physicsDt();
        for (ArenaParticipant p : participants) {
            p.prePhysics(this, dt);
        }
        long p0 = System.nanoTime();
        world.step(dt, config.box3dSubSteps());
        long p1 = System.nanoTime();
        gamePieces.syncFromWorld();
        for (ArenaParticipant p : participants) {
            p.postPhysics(this, dt);
        }
        double current = 0;
        for (ElectricalLoad load : loads) {
            current += load.supplyCurrent();
        }
        battery.update(current, dt);
        time += dt;
        lastStats = new ArenaStats(p1 - p0, System.nanoTime() - t0, gamePieces.awakeCount());
    }

    /** Runs ceil(seconds / physicsDt) steps. */
    public void advance(double seconds) {
        long steps = (long) Math.ceil(seconds / config.physicsDt() - 1e-9);
        for (long i = 0; i < steps; i++) {
            step();
        }
    }

    /** Calls reset on every participant, then resets the battery. */
    public void reset() {
        for (ArenaParticipant p : participants) {
            p.reset(this);
        }
        battery.reset();
    }

    public ArenaStats lastStepStats() {
        return lastStats;
    }

    @Override
    public void close() {
        world.close();
    }
}
