# Step 07: Arena loop and game pieces

**Goal:**

- The top-level simulation object (`Arena`) that owns the world, battery and participants and steps them in a
  fixed order.
- Game pieces as real 3D rigid bodies with air drag and Magnus lift. They replace maple-sim's analytic projectile
  and its 2D floor pieces.

**Depends on:** steps 04, 05, 06.

In this step, make `SwerveDriveSim` implement `ArenaParticipant`: `prePhysics(arena, dt)` calls `prePhysics(dt)`
and `postPhysics(arena, dt)` calls `postPhysics(dt)`.

## `box3dsim.arena`

```java
public interface ArenaParticipant {
    /** Before the world step: read inputs, apply forces. */
    default void prePhysics(Arena arena, double dt) {}
    /** After the world step: read results, handle sensor events, spawn or remove bodies. */
    default void postPhysics(Arena arena, double dt) {}
    /** Field reset (new match). */
    default void reset(Arena arena) {}
}

public record ArenaConfig(WorldConfig world, double physicsDt, int box3dSubSteps) {
    public static ArenaConfig defaults();     // WorldConfig.defaults(), 0.004 s, 4
}

public final class Arena implements AutoCloseable {
    public Arena(ArenaConfig config);
    public SimWorld world();
    public Battery battery();
    public GamePieces gamePieces();
    public double time();                     // seconds of simulated time since construction or reset
    public void addParticipant(ArenaParticipant p);   // stepped in insertion order
    public void addLoad(ElectricalLoad load);
    /** One physics step of config.physicsDt. */
    public void step();
    /** Runs ceil(seconds / physicsDt) steps. Robot integration calls step() itself per substep instead. */
    public void advance(double seconds);
    public void reset();                      // calls reset(arena) on every participant, then battery.reset()
    public ArenaStats lastStepStats();         // wall-clock nanos for physics and total, awake piece count
    @Override public void close();
}
```

`step()` order (fixed; tests depend on it):

1. Each participant's `prePhysics(this, dt)`, in insertion order. Robot participants first, then the field/rules
   participant, then `GamePieces` (aerodynamics). `Arena` adds `GamePieces` itself as the **last** participant at
   construction; later `addParticipant` calls insert before it.
2. `world.step(dt, box3dSubSteps)`.
3. `GamePieces.syncFromWorld()` (pose cache from `bodyMoves()`).
4. Each participant's `postPhysics(this, dt)`, in insertion order.
5. `battery.update(Σ load.supplyCurrent(), dt)`.
6. `time += dt`.

## `box3dsim.gamepiece`

```java
public record GamePieceType(String name, double radius, double mass, Material material,
                            double dragCoefficient,     // C_d
                            double liftSlope,           // C_L = min(liftMax, liftSlope · S), S = r|ω|/|v|
                            double liftMax,
                            double spinDecayPerSecond,  // exponential decay rate of |ω| while airborne
                            double linearDamping, double angularDamping) {}
```

`GamePieceType` holds spheres only in v0. Non-spherical pieces (coral, notes) would need a shape field later; leave
that out.

```java
public final class GamePiece {
    public int id();                 // stable, unique per Arena, never reused
    public GamePieceType type();
    public Pose3 pose();             // cached; updated from body moves
    public boolean isAwake();
    BodyHandle body(); ShapeHandle shape();   // package-private
}

public final class GamePieces implements ArenaParticipant {
    public GamePiece spawn(GamePieceType type, Vec3 position, Vec3 velocity, Vec3 angularVelocity, boolean awake);
    public void remove(GamePiece piece);           // destroys the body; the piece object becomes dead
    public Optional<GamePiece> byShape(ShapeHandle shape);
    public Collection<GamePiece> all();            // live pieces, unmodifiable view
    public List<Pose3> poses(GamePieceType type);  // fresh list, live pieces only
    public int count(GamePieceType type);
    public void clear();
    public double airDensity();                    // 1.204 kg/m³
    public void setAerodynamicsEnabled(boolean on);  // default on
    public void setFieldBounds(double minX, double maxX, double minY, double maxY);  // default: unbounded (±∞)
    public int outOfFieldCount();                  // pieces removed for leaving the field since last reset
}
```

### `spawn`

1. Create the body: `BodySpec.dynamic(pose).withVelocity(v, ω).withDamping(type.linearDamping, type.angularDamping)
   .withAwake(awake).withBullet(speed > 5 m/s)`.
2. Add the shape: `Sphere(ZERO, radius)` with `ShapeOptions.solid(material, density, GAME_PIECE)`. Density =
   mass / (4/3·π·r³); sensor events are on via the `GAME_PIECE` default.
3. Register `shape → piece` and `body → piece` maps.
4. Add the piece to the awake set if awake.

### `syncFromWorld()`

For each `BodyMove` in `world.bodyMoves()` whose body belongs to a piece: update the cached pose. If `fellAsleep`,
remove the piece from the awake set; otherwise add it. Ignore bodies that aren't pieces (robots).

### Aerodynamics (`prePhysics`, when enabled)

For each awake piece:

- Read `v` and `ω`. If |v| < 0.5 m/s, skip it. Rolling balls don't need drag.
- **Drag:** `F_d = −½·ρ·C_d·A·|v|·v`, with `A = π r²`.
- **Magnus:** `S = r·|ω|/|v|`, `C_L = min(liftMax, liftSlope·S)`, direction `n = normalize(ω × v)` (skip if
  |ω × v| < 1e-9), `F_L = ½·ρ·A·C_L·|v|²·n`.
- `applyForceToCenter(body, F_d + F_L)`.
- **Spin decay** while airborne: skip if z < radius + 0.01 (rolling on the floor; Box3D rolling resistance handles
  that). Otherwise `ω *= exp(−spinDecayPerSecond·dt)` and set it back with `setVelocity(body, v, ω')`.

### Out of field (`postPhysics`)

Any awake piece with z < −0.5, or x/y more than 1.0 m outside the field bounds, is removed and counted in
`outOfFieldCount`. Game rule G405 (ejecting fuel off the field) is the rules layer's business; it reads the count.

### `reset`

Reset `outOfFieldCount` only. **Do not clear pieces here.** `GamePieces` is the last participant, so clearing would
delete whatever a season participant (step 09) just spawned in its own `reset`. Seasons call `clear()` themselves
before respawning.

Out-of-field removal also applies to z < −0.5 when bounds are unbounded.

## Tests: `src/test/java/box3dsim/gamepiece/`

Use an `Arena` with a static ground (z top = 0, restitution 0, friction 0.8). Ball type: r = 0.075, m = 0.215,
material (0.6, 0.45, 0.05), C_d 0.5, liftSlope 0.5, liftMax 0.35, spin decay 0.3, damping 0, 0.

1. **Vacuum parabola:** aerodynamics off, launch from (0,0,1) at 8 m/s, 45°. Land at the analytic range
   `R = v cosθ/g·(v sinθ + sqrt(v² sin²θ + 2 g·(1 − r)))` ± 2 cm, measured where the ball first reaches z = r.
2. **Drag shortens range:** the same launch with drag on (C_d 0.5, no spin) lands 5–20% shorter than test 1.
3. **Backspin lengthens range:** the same launch with drag on and backspin |ω| = v/r about the axis that makes
   `ω × v` point up (for velocity along +x, ω along −y) lands farther than test 2.
4. **Sleep:** a ball dropped at rest falls asleep within 3 s, then drops out of the awake set, and `poses()`
   still returns its resting pose (z ≈ 0.075).
5. **Out of field:** with bounds (0, 10, 0, 10), a ball launched to land at x = 12 is removed and counted.
6. **Ids are unique:** spawn, remove, spawn gives different ids, and `byShape` of a removed piece's shape is empty.
7. **Pile:** spawn 400 balls on a 20 × 20 grid at 0.16 m pitch, z = 0.075, asleep. Then spawn one ball at 10 m/s
   into the grid. After 2 s, every z is within [0.07, 0.6] (no explosion and no tunneling through the floor), and
   `lastStepStats` reports an average physics step time. Print it. Don't assert a time; report it.

## Acceptance

- `.\gradlew.bat test` passes.
- `SwerveDriveSim` works through `Arena` (port one step-06 test to use `Arena.advance`).

Commit: `Add arena loop and 3D game pieces with drag and Magnus lift`.
