# Step 10: Intake, storage and shooter mechanisms

**Goal:** generic, motor-driven mechanisms that move game pieces between the world and a robot:

- a ground intake that takes FUEL off the carpet only when deployed and spinning;
- a storage path (hopper → indexer → feeder) that stages pieces;
- a flywheel and hood shooter that launches real FUEL bodies and loses flywheel energy on every shot.

**Depends on:** steps 05, 06, 07. These classes know nothing about REBUILT; step 11 wires them to the 2026-Offseason
robot and to `RebuiltMatch`.

Package `box3dsim.mechanism`.

## Shared ideas

- **Roller.** A `LumpedMechanism` driving a roller of radius `r`. Its surface speed is
  `v_s = velocity()·r·directionSign`. `directionSign` maps the robot's sign convention ("negative volts = intake",
  for example) so that `v_s > 0` always means "moving the piece inward or toward the shooter".
- **Rate limits.** Every transfer uses a token bucket: `tokens = min(1, tokens + dt·rate)`, where
  `rate = efficiency·v_s / pitch` pieces per second, pitch = piece diameter × 1.1. A transfer happens when
  `tokens ≥ 1`, and it spends one token. Faster rollers move pieces faster, and stopped rollers move none.
- **Energy.** Every transfer takes the piece's kinetic energy at the roller surface speed out of the roller with
  `removeEnergy(½·m·v_s²)`. That's small for intake and feeder rollers but it keeps the model honest. The big
  energy loss is the shooter's flywheel (below).

## `RollerConfig`

```java
public record RollerConfig(LumpedMechanism.Config mechanism, double rollerRadius, double directionSign,
                           double efficiency, double minSurfaceSpeed) {}
```

## `IntakeSim implements ArenaParticipant, ElectricalLoad`

```java
public record IntakeConfig(
        RollerConfig roller,
        LumpedMechanism.Config pivot,           // deploy mechanism; position in mechanism rad
        double deployedPosition,                // pivot position counted as fully deployed
        double stowedPosition,
        double deployedFraction,                // 0..1; intake works when progress ≥ this
        Vec3 zoneCenter, Vec3 zoneHalfExtents,  // sensor box in the ROBOT frame, where the deployed roller sweeps
        GamePieceType pieceType) {}

public IntakeSim(Arena arena, BodyHandle chassis, Battery battery, IntakeConfig config, PieceSink sink);
public void setRollerInput(double volts, boolean coastAtZero);
public void setPivotInput(double volts, boolean coastAtZero);
public LumpedMechanism roller(); public LumpedMechanism pivot();
public double deployProgress();                 // (pos − stowed)/(deployed − stowed), clamped 0..1
public int acquiredCount();                     // since reset
```

```java
/** Destination for acquired pieces. FuelStorage implements it. */
public interface PieceSink {
    boolean canAccept();
    void accept(GamePieceType type);
}
```

- **Construction:** add a sensor `Box(zoneHalfExtents, Pose3(zoneCenter))` to the chassis body with
  `ShapeOptions.sensor(SENSOR)`. Shapes on the same body are skipped by Box3D's sensor query, so the bumper never
  shows up as an overlap. Box3D queries the static, kinematic and dynamic trees, sleeping bodies included, so
  resting FUEL is detected.
- **`prePhysics(dt)`:** step the roller and pivot `LumpedMechanism`s with their inputs.
- **`postPhysics(dt)`:**
  1. Refill tokens using the roller's surface speed.
  2. If `deployProgress() ≥ deployedFraction`, `v_s ≥ minSurfaceSpeed` and `sink.canAccept()`, take
     `world.sensorOverlaps(zoneSensor)`, keep the shapes that are live pieces of `pieceType`, and sort them by
     distance to the zone center.
  3. For each piece while `tokens ≥ 1` and the sink can accept: remove it from the world, `sink.accept(type)`,
     spend a token, call `roller.removeEnergy(½·m·v_s²)`, and increment `acquiredCount`.
- **Not modeled in v0:** the deployed intake has no solid collider. FUEL in the zone is either taken or passes
  through, and the bumper still pushes FUEL as usual. A real roller bar that knocks FUEL away when the intake is
  stopped is a later improvement; note it in step 12.

## `FuelStorage implements ArenaParticipant, ElectricalLoad, PieceSink`

A hopper count, an indexer roller that moves pieces from the hopper into one staged slot, and a feeder roller that
hands the staged piece to the shooter.

```java
public record StorageConfig(int capacity, int preload, RollerConfig indexer, RollerConfig feeder,
                            GamePieceType pieceType) {}

public FuelStorage(Battery battery, StorageConfig config, Launcher launcher);
public void setIndexerInput(double volts, boolean coastAtZero);
public void setFeederInput(double volts, boolean coastAtZero);
public int hopperCount(); public boolean staged(); public int totalCount();   // hopper + staged
public LumpedMechanism indexer(); public LumpedMechanism feeder();
@Override public boolean canAccept();   // totalCount < capacity
@Override public void reset(Arena a);   // hopper = preload, staged = false, tokens 0
```

```java
/** Takes a staged piece. Returns false if it can't launch now (flywheel too slow); the piece stays staged. */
public interface Launcher { boolean launch(GamePieceType type); }
```

- **`prePhysics(dt)`:** step both rollers.
- **`postPhysics(dt)`:**
  1. **Indexer:** if `!staged && hopper > 0` and the indexer's `v_s ≥ minSurfaceSpeed`, refill the indexer bucket;
     on a token, `hopper--` and `staged = true`. If the indexer runs backward (`v_s ≤ −minSurfaceSpeed`) and staged,
     unstage (`staged = false`, `hopper++`).
  2. **Feeder:** if `staged` and the feeder's `v_s ≥ minSurfaceSpeed`, refill the feeder bucket. On a token, call
     `launcher.launch(type)`. If it returns true, `staged = false`. If it returns false, keep the token so the shot
     happens as soon as the flywheel is ready.
     - If the feeder runs backward while staged, unstage into the hopper.
     - In the 2026-Offseason code the indexer and feeder both run together while the flywheel is in tolerance,
       so pieces flow continuously.

## `ShooterSim implements ArenaParticipant, ElectricalLoad, Launcher`

```java
public record ShooterConfig(
        LumpedMechanism.Config flywheel,        // position/velocity in flywheel rad
        double flywheelRadius,
        double shotDirectionSign,               // sign of flywheel velocity that shoots (−1 for 2026-Offseason)
        double exitSpeedRatio,                  // v_exit = ratio · |ω|·r       (≈ 0.5 for a single wheel against a hood)
        double spinRatio,                       // |ω_ball| = spinRatio · v_exit / r_ball (backspin)
        double energyEfficiency,                // flywheel loses E_ball / efficiency per shot
        double minExitSpeed,                    // below this the launcher refuses
        LumpedMechanism.Config hood,            // position in hood mechanism rad
        DoubleUnaryOperator hoodPositionToPitch,// hood mechanism rad → launch pitch above horizontal, rad
        Vec3 exitPointRobot,                    // launch point in the ROBOT frame; must be outside robot collision boxes
        double exitYawRobot,                    // launch direction yaw in the ROBOT frame (0 = robot +x)
        GamePieceType pieceType) {}

public ShooterSim(Arena arena, SwerveDriveSim drive, Battery battery, ShooterConfig config, LaunchListener listener);
public void setFlywheelInput(double volts, boolean coastAtZero);
public void setHoodInput(double volts, boolean coastAtZero);
public LumpedMechanism flywheel(); public LumpedMechanism hood();
public double launchPitch();                    // current, rad
public int shotsFired();
public Optional<Vec3> lastExitVelocity();       // field frame, for logging
```

```java
public interface LaunchListener { void onLaunch(Pose3 robotPose, Vec3 exitVelocity); }
```

- **`prePhysics(dt)`:** step the flywheel and hood.
- **`launch(type)`:**
  1. Surface speed: `v_s = shotDirectionSign·flywheel.velocity()·flywheelRadius`. If
     `exitSpeedRatio·v_s < minExitSpeed`, return false.
  2. Exit speed and pitch: `v_exit = exitSpeedRatio·v_s` and `θ = hoodPositionToPitch(hood.position())`.
  3. Robot-frame direction: `d_r = (cos θ·cos y0, cos θ·sin y0, sin θ)`, with y0 = `exitYawRobot`. Rotate it by the
     chassis yaw into the field frame (`d`).
  4. Spawn point `p = drivePose.transformPoint(exitPointRobot)`. Its velocity adds the chassis's point velocity:
     `v = v_chassis + Ω ẑ × (p − c) + v_exit·d`.
  5. Backspin: axis `a = normalize(d × ẑ)`, which points along −y for a +x shot, as step 07 test 3 checks. Then
     `ω = spinRatio·v_exit/r_ball · a`.
  6. `gamePieces.spawn(type, p, v, ω, true)`.
  7. Energy: `E = ½·m·v_exit² + ½·(2/5·m·r²)·|ω|²`, then `flywheel.removeEnergy(E / energyEfficiency)`.
  8. `listener.onLaunch(drive.pose(), v)`, `shotsFired++`, return true.

## Tests: `src/test/java/box3dsim/mechanism/`

Arena with ground only, plus a `SwerveDriveSim` from `TestConfigs.maxSwerve()` (robot height 0.55). The piece is a
test FUEL type (r = 0.075, m = 0.215).

**IntakeSimTest**

The zone is in front of the front bumper: center (0.55, 0, 0.075), half extents (0.10, 0.30, 0.10). The pivot is
deployed at −2.0 rad and stowed at 0.

1. **Stowed:** a ball resting in the zone is not taken, even with the roller at 12 V.
2. **Deployed:** drive the pivot to −2.0 with a negative voltage (hard limit at −2.0). With the roller at 12 V and
   directionSign +1, the ball is taken within 0.5 s and `sink.accept` is called once.
3. **Rate:** 10 balls in the zone, a slow roller (2 V), and a fast one (12 V). The fast case takes them in less
   than half the time.
4. **Capacity:** with a sink of capacity 3, at most 3 are taken.
5. **Sleeping:** a ball created asleep in the zone is still taken (Box3D sensors see sleeping bodies).

**FuelStorageTest**

1. With preload 8, an indexer and feeder at full feed speed, and a launcher that always accepts, all 8 launch.
   The interval between launches is bounded below by the slower roller's rate.
2. With a launcher that refuses, one piece stays staged and the hopper count drops by exactly 1.
3. Running the feeder backward while staged returns the piece to the hopper.

**ShooterSimTest**

The flywheel is a Vortex 1:1, J = 0.004, r = 0.0508, shotDirectionSign −1, exit ratio 0.48, spin 0.8, efficiency
0.7. The hood is fixed at a 60° pitch (`hoodPositionToPitch = p → 60°`). The exit point is (0.1, 0, 0.64), yaw 0,
so the FUEL's bottom (0.565) clears the 0.55 m superstructure.

1. **Launch:** spin the flywheel at −12 V for 2 s, then `launch`. A ball spawns at the exit point with speed
   ≈ 0.48·|ω_before|·r ± 2% and pitch 60° ± 0.5°. ω_before is the flywheel speed just before the shot.
2. **Flywheel drop:** after the shot, |ω| equals `sqrt(ω_before² − 2E/(0.7·J))` within 1%.
3. **Recovery:** after a 5-shot burst at 0.1 s spacing under 12 V, the flywheel recovers to within 5% of its
   pre-burst speed within 1 s. This is the property real shooter tuning depends on.
4. **Moving robot:** with the robot driving at +2 m/s, the ball's field vx exceeds the stationary case by 2 m/s ± 0.05.
5. **Refusal:** with the flywheel stopped, `launch` returns false.

## Acceptance

- `.\gradlew.bat test` passes.

Commit: `Add intake, storage and shooter mechanisms`.
