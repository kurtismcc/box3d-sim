# Step 04: SimWorld API and Box3D backend

**Goal:** a small, backend-neutral physics API (`SimWorld`) implemented on Box3D. Everything above this layer
(drivetrain, game pieces, field, mechanisms) uses only this API and never touches `box3dsim.ffm.gen`.

**Depends on:** step 03.

## Conventions (whole library)

- SI units: meters, kilograms, seconds, radians, newtons, volts, amps.
- **Field frame = WPILib field frame:** origin at the blue alliance wall's right corner, +x toward the red alliance,
  +y to the left from blue's view, **+z up**. Gravity is (0, 0, −9.81).
- Doubles in all Java APIs. Box3D is single precision; convert at the FFM boundary only.

## `box3dsim.math`

Immutable records with no WPILib dependency.

```java
public record Vec3(double x, double y, double z) {
    public static final Vec3 ZERO = new Vec3(0, 0, 0);
    public Vec3 plus(Vec3 o); public Vec3 minus(Vec3 o); public Vec3 times(double s);
    public double dot(Vec3 o); public Vec3 cross(Vec3 o); public double norm(); public double normSquared();
    public Vec3 normalized();             // ZERO stays ZERO
}

public record Quat(double w, double x, double y, double z) {
    public static final Quat IDENTITY = new Quat(1, 0, 0, 0);
    public static Quat fromAxisAngle(Vec3 axis, double angle);
    public static Quat fromYaw(double yaw);                    // rotation about +z
    public static Quat fromRollPitchYaw(double roll, double pitch, double yaw); // extrinsic X then Y then Z, same as WPILib Rotation3d(roll, pitch, yaw)
    public Quat times(Quat o);            // Hamilton product, this * o
    public Quat inverse();                // conjugate of the normalized quaternion
    public Vec3 rotate(Vec3 v);
    public double yaw();                  // atan2(2(wz + xy), 1 − 2(y² + z²))
    public Quat normalized();
}

public record Pose3(Vec3 position, Quat rotation) {
    public static final Pose3 IDENTITY = new Pose3(Vec3.ZERO, Quat.IDENTITY);
    public static Pose3 ofXYYaw(double x, double y, double z, double yaw);
    public Vec3 transformPoint(Vec3 local);   // rotation.rotate(local) + position
    public Vec3 transformVector(Vec3 local);  // rotation.rotate(local)
    public Vec3 inverseTransformPoint(Vec3 world);
    public Pose3 times(Pose3 local);          // compose: this ∘ local
}
```

`MathTest` covers:

- `fromYaw(π/2).rotate((1,0,0)) == (0,1,0)`
- `fromRollPitchYaw(r, p, y).yaw() == y` for |pitch| < π/2
- `Pose3.times` composition
- round trip `inverseTransformPoint(transformPoint(v)) == v`

Use tolerance 1e-9.

## `box3dsim.world` types

```java
public enum BodyType { STATIC, KINEMATIC, DYNAMIC }

/** Opaque handles. The long holds the 8-byte b3BodyId / b3ShapeId bit pattern. */
public record BodyHandle(long id) {}
public record ShapeHandle(long id) {}

public record Material(double friction, double restitution, double rollingResistance) {
    public static final Material DEFAULT = new Material(0.6, 0.0, 0.0);
}

public record MotionLocks(boolean linearX, boolean linearY, boolean linearZ,
                          boolean angularX, boolean angularY, boolean angularZ) {
    public static final MotionLocks NONE = new MotionLocks(false, false, false, false, false, false);
    /** Planar chassis: locks z translation, roll and pitch. */
    public static final MotionLocks PLANAR = new MotionLocks(false, false, true, true, true, false);
}

public record CollisionFilter(long categoryBits, long maskBits) {}
```

### `CollisionLayers` (final class of constants)

| Layer | Bit | Collides with |
|---|---|---|
| `FIELD` | 1L<<0 | everything except `FIELD` |
| `ROBOT` | 1L<<1 | `FIELD`, `ROBOT`, `GAME_PIECE`, `ROBOT_BLOCKER` |
| `GAME_PIECE` | 1L<<2 | `FIELD`, `ROBOT`, `GAME_PIECE`, `PIECE_BLOCKER`, `SENSOR` |
| `ROBOT_BLOCKER` | 1L<<3 | `ROBOT` only (invisible walls that stop robots, not balls) |
| `PIECE_BLOCKER` | 1L<<4 | `GAME_PIECE` only (field geometry that robots drive through, such as BUMP ramps in v0) |
| `SENSOR` | 1L<<5 | `GAME_PIECE` only |

Provide `public static CollisionFilter of(long layerBit)`, which returns the filter with that category and the
mask from the table. Box3D only creates a contact when
`(a.category & b.mask) != 0 && (b.category & a.mask) != 0`, so keep the table symmetric.

### Shapes

```java
public sealed interface ShapeSpec {
    record Sphere(Vec3 center, double radius) implements ShapeSpec {}
    record Capsule(Vec3 p1, Vec3 p2, double radius) implements ShapeSpec {}            // segment p1–p2, body-local
    record Box(Vec3 halfExtents, Pose3 localPose) implements ShapeSpec {}
    record Hull(List<Vec3> points, Pose3 localPose) implements ShapeSpec {}            // convex, ≤ 128 points
    record Mesh(List<Vec3> vertices, int[] triangleIndices) implements ShapeSpec {}   // STATIC bodies only, CCW
}

public record ShapeOptions(Material material, double density, CollisionFilter filter,
                           boolean isSensor, boolean enableSensorEvents, boolean updateBodyMass) {
    public static ShapeOptions solid(Material m, double density, long layerBit);   // updateBodyMass = true
    public static ShapeOptions sensor(long layerBit);   // isSensor, enableSensorEvents, density 0, updateBodyMass = FALSE
    public ShapeOptions withSensorEvents(boolean enable);
    public ShapeOptions withUpdateBodyMass(boolean update);
}
```

`updateBodyMass = false` matters. `b3Body_SetMassData` says its override "is lost if a shape is added or removed"
when the new shape updates body mass. Sensors attached later to a chassis whose mass was overridden (step 10's
intake zone) must not reset it.

```java
```

### Bodies

```java
public record BodySpec(BodyType type, Pose3 pose, Vec3 linearVelocity, Vec3 angularVelocity,
                       double linearDamping, double angularDamping, double gravityScale,
                       boolean bullet, boolean enableSleep, boolean awake, MotionLocks locks, String name) {
    public static BodySpec dynamic(Pose3 pose);   // zero velocities, damping 0, gravityScale 1, sleep on, awake, NONE locks
    public static BodySpec fixed(Pose3 pose);     // STATIC
    public BodySpec withLocks(MotionLocks l); public BodySpec withBullet(boolean b);
    public BodySpec withAwake(boolean a);         // false = created asleep (resting game pieces at match start)
    public BodySpec withGravityScale(double g); public BodySpec withDamping(double lin, double ang);
    public BodySpec withVelocity(Vec3 lin, Vec3 ang); public BodySpec withName(String n);
}
```

### Events and queries

```java
public record SensorEvent(ShapeHandle sensor, ShapeHandle visitor) {}
public record BodyMove(BodyHandle body, Pose3 pose, boolean fellAsleep) {}
public record RayHit(ShapeHandle shape, Vec3 point, Vec3 normal, double fraction) {}
```

## `SimWorld` interface

```java
public interface SimWorld extends AutoCloseable {
    BodyHandle createBody(BodySpec spec);
    void destroyBody(BodyHandle body);                       // also destroys its shapes
    ShapeHandle addShape(BodyHandle body, ShapeSpec shape, ShapeOptions options);
    BodyHandle bodyOf(ShapeHandle shape);

    /** Advances dt seconds; Box3D splits it into subSteps solver substeps. */
    void step(double dt, int subSteps);

    Pose3 pose(BodyHandle body);
    void setPose(BodyHandle body, Pose3 pose);               // teleport; wakes the body
    Vec3 linearVelocity(BodyHandle body);
    Vec3 angularVelocity(BodyHandle body);
    void setVelocity(BodyHandle body, Vec3 linear, Vec3 angular);
    Vec3 pointVelocity(BodyHandle body, Vec3 worldPoint);

    void applyForce(BodyHandle body, Vec3 force, Vec3 worldPoint); // wake = true
    void applyForceToCenter(BodyHandle body, Vec3 force);
    void applyTorque(BodyHandle body, Vec3 torque);

    /** Overrides shape-derived mass. inertiaDiagonal is about the local center, in body axes. */
    void setMassProperties(BodyHandle body, double mass, Vec3 localCenter, Vec3 inertiaDiagonal);
    double mass(BodyHandle body);

    boolean isAwake(BodyHandle body);
    void setEnabled(BodyHandle body, boolean enabled);       // disabled bodies leave the simulation but keep their ids

    /** Valid until the next step(). */
    List<SensorEvent> sensorBeginEvents();
    List<SensorEvent> sensorEndEvents();
    List<BodyMove> bodyMoves();
    /** Shapes currently overlapping a sensor shape (polling, independent of events). */
    List<ShapeHandle> sensorOverlaps(ShapeHandle sensor);

    Optional<RayHit> castRayClosest(Vec3 origin, Vec3 translation, CollisionFilter filter);

    @Override void close();                                  // destroys the world; idempotent
}
```

```java
public record WorldConfig(Vec3 gravity, int workerCount, boolean enableSleep, boolean enableContinuous,
                          double restitutionThreshold, double maximumLinearSpeed) {
    public static WorldConfig defaults(); // (0,0,-9.81), 1 worker, sleep on, continuous on, 0.5 m/s, 30 m/s
}
```

`workerCount = 1` keeps runs deterministic. `restitutionThreshold = 0.5` m/s lets slow balls bounce a little; Box3D's
default is higher.

## `Box3dWorld implements SimWorld`

Constructor: `public Box3dWorld(WorldConfig config)`. It calls `NativeLibrary.load()`, then
`b3DefaultWorldDef` and sets `gravity`, `workerCount`, `enableSleep`, `enableContinuous`, `restitutionThreshold`
and `maximumLinearSpeed`, then `b3CreateWorld`.

### Memory

- `Arena arena = Arena.ofShared()` lives as long as the world. It holds the world id, mesh data (meshes keep
  pointers to their input) and a 64 KiB scratch segment.
- **Scratch allocator:** a private `ScratchAllocator implements SegmentAllocator` that bump-allocates from the scratch
  segment, honoring the requested alignment. Every public method calls `scratch.reset()` first. All temporary
  structs (definitions, vectors passed or returned by value, event copies) come from it. If one call needs more than
  64 KiB (a large mesh), allocate from `arena` instead.
- Don't keep `MemorySegment`s across public calls, except in the long-lived `arena`.

### Ids

- `b3BodyId` and `b3ShapeId` are 8 bytes: `{int32 index1; uint16 world0; uint16 generation}`. Read the whole struct
  as one `long` with `seg.get(ValueLayout.JAVA_LONG_UNALIGNED, 0)` to get the handle.
- To pass an id by value, allocate 8 bytes from scratch and write the long back.
- `b3WorldId` is 4 bytes. Keep its segment in `arena`.

### Conversions

- `Vec3` is written to and read from `b3Vec3` (3 floats).
- `Quat` maps to `b3Quat`, which is `{b3Vec3 v; float s}`: **vector part first, scalar last**. So Box3D order is
  (x, y, z, w).
- `Pose3` maps to `b3WorldTransform`, which is `{b3Pos p; b3Quat q}`. `b3Pos` is `b3Vec3` in the single-precision
  build.

### Shapes

| ShapeSpec | Box3D call |
|---|---|
| `Sphere` | `b3Sphere{center, radius}` then `b3CreateSphereShape` |
| `Capsule` | `b3Capsule{center1, center2, radius}` then `b3CreateCapsuleShape` |
| `Box` | `b3MakeTransformedBoxHull(hx, hy, hz, b3Transform(localPose))`; pass the returned `b3BoxHull` segment as the hull pointer to `b3CreateHullShape` |
| `Hull` | write the points to a `b3Vec3[]` in scratch, `b3CreateHull(points, n, 64)`, then `b3CreateTransformedHullShape(body, def, hull, transform, scale=(1,1,1))`, then `b3DestroyHull(hull)`. The world database clones it |
| `Mesh` | there is no `b3DefaultMeshDef` at the pinned commit, so `b3MeshDef.allocate(...)` (zeroed) and set every field: `vertices`, `stride = 12`, `indices` (int32 triples), `materialIndices = NULL`, `weldTolerance = 0`, `vertexCount`, `triangleCount`, `weldVertices = false`, `useMedianSplit = false`. Call `b3CreateMesh(def, NULL, 0)`, then `b3CreateMeshShape(body, def, mesh, scale=(1,1,1))`. **Allocate the vertex/index arrays and keep the returned `b3MeshData*` in `arena`**, and call `b3DestroyMesh` in `close()` after `b3DestroyWorld` |

Shape definition: start from `b3DefaultShapeDef`, then set:

- `baseMaterial.friction`, `.restitution`, `.rollingResistance`
- `density`
- `filter.categoryBits`, `.maskBits` (`groupIndex` stays 0)
- `isSensor`, `enableSensorEvents`
- `updateBodyMass` from the options

**Sensor events need `enableSensorEvents = true` on both the sensor and the visitor.** `ShapeOptions.solid(...)`
must therefore default `enableSensorEvents` to `true` for the `GAME_PIECE` layer and `false` otherwise.

### Body creation

From `b3DefaultBodyDef`, set:

- `type` (`b3_staticBody`=0, `b3_kinematicBody`=1, `b3_dynamicBody`=2)
- `position`, `rotation`, `linearVelocity`, `angularVelocity`
- `linearDamping`, `angularDamping`, `gravityScale`
- `isBullet`, `enableSleep`, `isAwake`
- `motionLocks`, which has six bools in the order `linearX, linearY, linearZ, angularX, angularY, angularZ`

`name` is a `const char*`. Skip it (leave NULL), or allocate a C string in `arena`.

### Mass

`b3Body_SetMassData` with `b3MassData{float mass; b3Vec3 center; b3Matrix3 inertia}`. `b3Matrix3` is three column
vectors `cx, cy, cz`; for the diagonal, set `cx=(Ixx,0,0)`, `cy=(0,Iyy,0)`, `cz=(0,0,Izz)`. Shapes added later
recompute mass because of `updateBodyMass`. Callers therefore call `setMassProperties` **after** adding all shapes.

### Events (copy out right after `b3World_Step`)

- `b3World_GetSensorEvents(scratch, world)` returns `{beginEvents*, endEvents*, beginCount, endCount}`. Each event
  is `{b3ShapeId sensorShapeId; b3ShapeId visitorShapeId}`.
- `b3World_GetBodyEvents(scratch, world)` returns `{b3BodyMoveEvent* moveEvents; int moveCount}`. Each event is
  `{void* userData; b3WorldTransform transform; b3BodyId bodyId; bool fellAsleep}`.
- Read the counts, reinterpret the pointers to `count * eventLayout.byteSize()`, and build immutable Java lists.
- Keep them in fields that `sensorBeginEvents()`, `sensorEndEvents()` and `bodyMoves()` return until the next
  step. Don't use `scratch` for these lists, because the scratch is reset on the next call; they are Java objects.

### Other calls

| Method | Box3D |
|---|---|
| `sensorOverlaps` | `n = b3Shape_GetSensorCapacity(shape)`, allocate `n` ids, `b3Shape_GetSensorData(shape, ids, n)` |
| `castRayClosest` | `b3World_CastRayClosest(world, origin, translation, b3QueryFilter{categoryBits, maskBits, id=0, name=NULL})`. Start the filter from `b3DefaultQueryFilter`. Read `b3RayResult` (`shapeId`, `point`, `normal`, `fraction`, `bool hit`; the other fields are diagnostics) |
| `setPose` | `b3Body_SetTransform(body, pos, rot)` then `b3Body_SetAwake(body, true)` |
| `setEnabled` | `b3Body_Enable` / `b3Body_Disable` |
| `close` | idempotent: `b3DestroyWorld`, destroy meshes, then `arena.close()` |

## Tests: `src/test/java/box3dsim/world/Box3dWorldTest.java`

Use `new Box3dWorld(WorldConfig.defaults())`. Static ground is a `Box((10,10,0.5), pose z=-0.5)` with `FIELD` layer
and material (0.8, 0.5, 0). Every test closes its world.

1. **Rest:** a 0.075 m sphere (`GAME_PIECE`, density for 0.215 kg) dropped from z=1 rests at z=0.075±0.005 after
   3 s at 200 Hz (`step(0.005, 4)`).
2. **Bounce:** the same sphere with restitution 0.5. Drop it from z=1.075 (1.0 m fall) and record the max z after the
   first contact. Expect a rebound height of e²·1.0 = 0.25 m ± 0.05. Box3D mixes restitution as the **max** of the
   two shapes and friction as the **geometric mean** √(μa·μb) (`physics_world.c` default callbacks).
3. **Sensor events:** a static body with a sensor `Box` (0.5 m half-extents) at z=2, and a sphere dropped from z=3.
   Exactly one begin event (sensor, ball) occurs while it falls through, and later exactly one end event.
   `sensorOverlaps` returns the ball while it is inside.
4. **Filtering:** a `ROBOT_BLOCKER` static box spanning the ball's path. The ball falls through it. A `ROBOT` dynamic
   box dropped onto the same blocker stops on it.
5. **Motion locks:** a dynamic box with `MotionLocks.PLANAR`, `gravityScale 0`, placed at z=0.3. Apply a force
   (100, 0, 0) at a point 0.2 m above its center for 1 s. Afterwards |z−0.3| < 1e-4, the roll/pitch angle magnitude is
   < 1e-4 rad, and x has increased.
6. **Body moves:** after the ball from test 1 falls asleep, `bodyMoves()` stops listing it, and one step had
   `fellAsleep == true` for it.
7. **Ray:** `castRayClosest((0,0,1), (0,0,-2), new CollisionFilter(GAME_PIECE, FIELD))` hits at z=0±1e-4 with
   normal (0,0,1). Box3D queries check both directions: `shape.category & query.mask` and
   `shape.mask & query.category` (`b3ShouldQueryCollide`). A query filter must therefore carry a category that
   the target layer's mask accepts. `CollisionLayers.of(FIELD)` would miss FIELD shapes.
8. **Capsule:** a dynamic capsule from (−0.2,0,0) to (0.2,0,0), radius 0.05, dropped from z = 1, rests at
   z = 0.05 ± 0.005 lying flat.
9. **Created asleep:** a sphere created with `withAwake(false)` resting on the ground (z = 0.075) has
   `isAwake == false` and produces no `bodyMoves()` entries for 10 steps. A second sphere dropped onto it wakes it.
10. **Mass override:** a dynamic box with `gravityScale 0` placed in the air at z = 2 (no contacts), then
   `setMassProperties(box, 50, ZERO, (1,1,6.883))`. Afterwards, add a sensor shape with
   `ShapeOptions.sensor(SENSOR)` and check that `mass(box)` is still 50. A torque of 6.883 N·m about z for 1 s gives
   ω_z ≈ 1.0 rad/s ± 2%.

## Acceptance

- `.\gradlew.bat test` passes, including all ten tests above and `MathTest`.
- No class outside `box3dsim.world.Box3dWorld` (and its private helpers) imports `box3dsim.ffm.gen`. Check with
  `git grep -l "box3dsim.ffm.gen" src/main/java`, which lists only `ffm/` and `world/Box3dWorld*.java`.

Commit: `Add SimWorld API and Box3D backend`.
