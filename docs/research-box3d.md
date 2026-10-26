# Box3D: what it offers and how to call it from Java

Researched 2026-10-26 from a shallow clone of https://github.com/erincatto/box3d at `f36d061` (2026-10-20). That
commit is the one step 02 pins.

## Status

- MIT license.
- Initial commit 2026-05-10. The only tag is `v0.1.0` (2026-06-30).
- The announcement (https://box2d.org/posts/2026/06/announcing-box3d/) calls it alpha software that needs more
  testing and documentation.
- Manual: https://box2d.org/documentation3d/
- The public headers changed a lot after `v0.1.0`: about 600 lines added and 340 removed in `include/`, with renames
  such as `b3CreateCompoundShape` → `b3CreateBakedCompoundShape` and `b3Body_GetLocalCenterOfMass` →
  `b3Body_GetLocalCenter`. **Pin a commit and regenerate the bindings on every bump.**

## API shape (same pattern as Box2D v3)

- Definitions come from `b3DefaultXxxDef()` by value. Every definition carries `internalValue`, a cookie that
  asserts check, so never zero-fill a definition.
- Handles are small value structs:

  | Handle | Layout | Size |
  |---|---|---|
  | `b3WorldId` | `{uint16 index1, generation}` | 4 bytes |
  | `b3BodyId`, `b3ShapeId`, `b3JointId` | `{int32 index1; uint16 world0; uint16 generation}` | 8 bytes |
  | `b3ContactId` | | 12 bytes |

- Functions are `b3World_*`, `b3Body_*`, `b3Shape_*`, `b3Joint_*`, exported with `B3_API`.
- Headers live in `include/box3d/`: `base.h`, `box3d.h`, `collision.h`, `config.h`, `constants.h`, `id.h`,
  `math_functions.h`, `types.h`.
- **Math types** (single-precision build):
  - `b3Vec3{float x,y,z}`
  - `b3Quat{b3Vec3 v; float s}`, vector part first
  - `b3Transform{p, q}`
  - `b3Matrix3{cx, cy, cz}`, columns
  - `b3Pos` = `b3Vec3` and `b3WorldTransform` = `b3Transform`, unless `BOX3D_DOUBLE_PRECISION`, which changes the ABI.
- **Not in the DLL:** about 140 `static inline` helpers (`math_functions.h`, `id.h`, `B3_IS_NULL`). Reimplement
  them in Java.

## Features box3d-sim uses

| Need | Box3D |
|---|---|
| World, gravity, step | `b3CreateWorld`, `b3World_Step(world, dt, subSteps)`; gravity defaults to (0,−10,0), and box3d-sim sets (0,0,−9.81) |
| Determinism | `workerCount = 1` |
| Bodies | static / kinematic / dynamic; `isBullet`; `enableSleep`; `isAwake`; `gravityScale`; damping |
| Planar chassis | `b3MotionLocks{linearX, linearY, linearZ, angularX, angularY, angularZ}` on the body definition or `b3Body_SetMotionLocks` |
| Mass override | `b3Body_SetMassData(b3MassData{mass, center, inertia})` |
| Shapes | sphere, capsule, hull (`b3CreateHull`, ≤ 128 vertices; boxes via `b3MakeBoxHull` / `b3MakeTransformedBoxHull`, which return `b3BoxHull` with the hull at offset 0), triangle mesh (static only; the shape keeps a pointer to the mesh data), height field |
| Materials | `b3SurfaceMaterial{friction, restitution, rollingResistance, tangentVelocity, userMaterialId, customColor}`. Default mixing: friction = √(a·b), restitution = max(a, b) (`physics_world.c`). Rolling resistance (spheres and capsules only) mixes as `max(a, b)·radius` and caps a rolling impulse at `rr·r·totalNormalImpulse` (`contact.c`, `contact_solver.c`). **Measured: any rr > 0, even 0.002, stops a sphere that starts sliding from ever spinning up, so it skids to rest. box3d-sim keeps it at 0 and applies rolling resistance as a force (step 07)** |
| Filtering | `b3Filter{uint64 categoryBits, maskBits; int groupIndex}` (64-bit) |
| Sensors | `isSensor` plus `enableSensorEvents` on both the sensor and the visitor (default false). Visitors must be convex. They are processed at the end of the step with no CCD. Begin and end events come from `b3World_GetSensorEvents`, and polling from `b3Shape_GetSensorData`. The sensor query covers the static, kinematic and dynamic trees, **sleeping bodies included**, and skips shapes on the sensor's own body (`sensor.c`) |
| Contacts | `enableContactEvents` defaults to false, unlike Box2D v3. Hit events need `enableHitEvents` plus a threshold. Per-body contact data comes from `b3Body_GetContactData`, and manifolds expose normal impulses |
| Moved bodies | `b3World_GetBodyEvents` returns `b3BodyMoveEvent{userData, transform, bodyId, fellAsleep}`, only for bodies that moved |
| Queries | `b3World_CastRayClosest(world, origin, translation, b3QueryFilter)` returns `b3RayResult{shapeId, point, normal, userMaterialId, fraction, triangleIndex, childIndex, nodeVisits, leafVisits, hit}` |
| Joints | revolute, prismatic, weld, wheel, distance, motor, spherical, parallel, filter. Not used in v0 |

## Build

- CMake with `BUILD_SHARED_LIBS=ON` produces `bin/box3d.dll`. CMake defines `box3d_EXPORTS`, which gives
  `__declspec(dllexport)`.
- The project declares `LANGUAGES C CXX`.
- Options: `BOX3D_SAMPLES`, `BOX3D_UNIT_TESTS`, `BOX3D_BENCHMARKS`, `BOX3D_DOCS`, `BOX3D_VALIDATE`,
  `BOX3D_DISABLE_SIMD`, `BOX3D_DOUBLE_PRECISION`.
- AVX2 is picked at runtime with an SSE2 fallback.
- `CMakePresets.json` has `windows`, `windows-clang-cl`, `clang-cl-release` (Ninja), `mingw-release`,
  `linux-release` and `macos`.
- The project's CI builds Windows DLLs with MSVC.

## Java FFM

- **jextract** `25-jextract+2-4` (2025-11-25, JDK 25 based, early access): https://jdk.java.net/jextract/
  - Generate without `-l`, so symbols resolve through `SymbolLookup.loaderLookup()` after `System.load`.
  - Filter generation to `B3_API` functions.
- **By-value structs** work in the Java 25 FFM API. Returned structs add a leading `SegmentAllocator`; passed
  structs are `MemorySegment`s.
- Event arrays are valid only until the next step, so copy them out.
- An exception in an upcall kills the JVM, and some callbacks run on worker threads. v0 uses no upcalls.

### Existing bindings (reference only; box3d-sim generates its own)

| Repo | License | Approach |
|---|---|---|
| `Plixo2/B3J` | MIT | jextract plus natives; around v0.1.0, stale API |
| `Morgoth398/Box3D-JavaFFM` | MIT | hand-written; needs Java 26 preview; has a natives CI workflow worth copying |
| `xpenatan/jBox3D` | Apache-2.0 | WebIDL, JNI and FFM; snapshot only |
| `Meekiavelique/box3d-java` | **no license** | do not copy |

## Fallback engine (only if Box3D proves unworkable)

Jolt via `com.github.stephengold:jolt-jni-Windows64:6.2.0` (MIT; natives for Windows, Linux and macOS, x64 and arm).
It is mature, but its API is C++-shaped through JNI. Only the `SimWorld` backend would change, which is why every
caller above step 04 goes through that interface.
