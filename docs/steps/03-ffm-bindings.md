# Step 03: Box3D FFM bindings

**Goal:** generated Java 25 FFM bindings for the Box3D C API, a loader that pulls the DLL out of the jar, and a smoke
test that drops a ball on a floor.

**Depends on:** steps 01, 02.

## Approach

- Generate bindings with **jextract**; don't hand-write struct layouts. Box3D definitions mix `float`, `bool`
  (1 byte), 64-bit masks and nested structs, and jextract computes padding correctly.
- Commit the generated sources under `src/main/java/box3dsim/ffm/gen/`. Regenerate whenever the Box3D pin changes.
  Hand-edit nothing in `gen/`.
- Generate **without** `-l`. jextract then resolves symbols through `SymbolLookup.loaderLookup()`, which finds any
  library loaded with `System.load(path)`. `NativeLibrary.load()` (below) must run before the first generated call.
- Bind only exported functions. About 140 helpers in `math_functions.h` and `id.h` are `static inline`
  (`b3Add`, `b3MakeQuat`, `B3_IS_NULL`, ...) and are not in the DLL. A downcall to one compiles, then fails at
  runtime. Write the few we need in Java instead (step 04).

## jextract

jextract early-access build **25-jextract+2-4** (2025-11-25, JDK 25 based): https://jdk.java.net/jextract/

### `scripts/get-jextract.ps1`

1. Download
   `https://download.java.net/java/early_access/jextract/25/2/openjdk-25-jextract+2-4_windows-x64_bin.tar.gz` and its
   `.sha256` into `tools/` (gitignored).
2. Verify the SHA-256 (`Get-FileHash -Algorithm SHA256`) and throw on mismatch.
3. Extract with `tar -xzf` into `tools/`. Windows 11 ships `tar`.
4. Print the path of `jextract.bat` (under `tools/jextract-25/bin/`; the exact folder name comes from the archive, so
   find it with `Get-ChildItem tools -Recurse -Filter jextract.bat`).

### `scripts/generate-bindings.ps1`

Run from the repo root.

1. Locate `jextract.bat` under `tools/`. Fail with "run scripts/get-jextract.ps1 first" if it's absent.
2. Dump the symbol list:

   ```
   jextract.bat --dump-includes native-build/jextract-all.txt -I native/box3d/include native/box3d/include/box3d/box3d.h
   ```

   Each line looks like `--include-function b3CreateWorld    # header: <path>`. There are also lines for
   `--include-struct`, `--include-typedef`, `--include-constant`, `--include-union` and `--include-var`.
3. Build the set of exported names: every identifier that directly follows `B3_API <return type>` in
   `native/box3d/include/box3d/*.h`. Use the regex `B3_API\s+[\w\s\*]+?\b(b3\w+)\s*\(` per line. Some
   declarations span lines, but the name is always on the `B3_API` line.
4. Write `native-build/jextract-includes.txt`, keeping:
   - every `--include-function` line whose name is in the exported set;
   - every other `--include-*` line whose `# header:` path, with backslashes normalized to `/`, contains
     `native/box3d/include`. This drops `stdint`/`math.h` symbols. Don't match on plain `box3d`: the repo
     folder is `box3d-sim`, so jextract's bundled clang headers under `tools/` would match too.
5. Delete `src/main/java/box3dsim/ffm/gen/` so stale files don't survive, then generate:

   ```
   jextract.bat --output src/main/java --target-package box3dsim.ffm.gen --header-class-name Box3D -I native/box3d/include "@native-build/jextract-includes.txt" native/box3d/include/box3d/box3d.h
   ```

6. Throw on any non-zero exit code.

If jextract reports that an included symbol depends on an excluded one, add the missing struct or typedef to the keep
rules. Don't switch to generating everything.

### What generated code looks like

This is the jextract 22+ style, so read the generated files to confirm:

- `Box3D` (it may also split into `Box3D_1`, `Box3D_2`, ...) has static methods per function, for example
  `Box3D.b3World_Step(MemorySegment worldId, float timeStep, int subStepCount)`. Constants appear as static methods
  too (`Box3D.b3_dynamicBody()`).
- **Struct returned by value:** the method takes a leading `SegmentAllocator`, for example
  `MemorySegment def = Box3D.b3DefaultWorldDef(arena);`.
- **Struct passed by value:** pass a `MemorySegment` holding the struct.
- **Struct class per C struct:** for example `b3WorldDef`, `b3Vec3`, `b3BodyId`. Each has `layout()`,
  `allocate(SegmentAllocator)`, a getter `gravity(MemorySegment struct)` (returning a slice for nested structs, or the
  value for primitives) and a setter `gravity(MemorySegment struct, MemorySegment value)` or `x(MemorySegment, float)`.
- `bool` maps to Java `boolean`, and `uint64_t` to `long`.

## `box3dsim.ffm.NativeLibrary`

```java
public final class NativeLibrary {
    /** Loads box3d once per JVM. Safe to call repeatedly and from any thread. */
    public static synchronized void load();
    /** Absolute path of the loaded library file. Only valid after load(). */
    public static Path loadedPath();
}
```

`load()` does the following:

1. Return if already loaded.
2. If system property `box3dsim.nativePath` is set, `System.load` that path. This is for debugging a locally built
   DLL.
3. Otherwise:
   - Map the platform. If `os.name` starts with `Windows` and `os.arch` is `amd64` or `x86_64`, the platform is
     `windows-x86_64`. Anything else throws `UnsupportedOperationException("box3d-sim v0 ships only windows-x86_64 natives")`.
   - Read resource `/natives/windows-x86_64/box3d.dll` and resource `/natives/box3d.version`. The version file holds
     the commit SHA; trim it.
   - Target path: `%TEMP%\box3dsim\<first 12 chars of SHA>-<SHA-256 of the DLL bytes, first 12 hex chars>\box3d.dll`.
     If the target exists with the same size, reuse it. Otherwise write it to a temp name in the same directory
     and `Files.move` it with `ATOMIC_MOVE`. A loaded DLL is locked on Windows; a content-addressed path never needs
     overwriting.
   - Call `System.load(target.toString())`.
4. After loading, check the build:
   - `Box3D.b3IsDoublePrecision()` must be `false`; otherwise throw `IllegalStateException` that names the cause
     (wrong `BOX3D_DOUBLE_PRECISION`).
   - Read `Box3D.b3GetVersion(arena)` into `major.minor.revision` and keep it for `version()`.

Also add `public static String version()`, which returns e.g. `"0.1.0 (f36d061ba269)"`.

## Smoke test: `src/test/java/box3dsim/ffm/Box3dSmokeTest.java`

This test uses the generated API directly, inside `try (Arena arena = Arena.ofConfined())`.

1. `NativeLibrary.load()`.
2. World:
   - `def = Box3D.b3DefaultWorldDef(arena)`.
   - Set `def.gravity` to (0, 0, −9.81). Write the three floats into the nested `b3Vec3` slice via
     `b3Vec3.x(b3WorldDef.gravity(def), 0f)` and so on.
   - Set `workerCount` to 1.
   - `world = Box3D.b3CreateWorld(arena, def)`. The return is a 4-byte `b3WorldId` struct, so it takes an allocator.
3. Ground:
   - `bodyDef = Box3D.b3DefaultBodyDef(arena)`. Its type is static by default. Position (0, 0, −0.5).
   - `ground = Box3D.b3CreateBody(arena, world, bodyDef)`.
   - `box = Box3D.b3MakeBoxHull(arena, 10f, 10f, 0.5f)`. `b3BoxHull` starts with its `b3HullData base`, so pass the
     segment itself (offset 0) as the `const b3HullData*`.
   - `shapeDef = Box3D.b3DefaultShapeDef(arena)`, then `Box3D.b3CreateHullShape(arena, ground, shapeDef, box)`.
4. Ball:
   - Body type `b3_dynamicBody()`, position (0, 0, 1.0).
   - Sphere with radius 0.075. Allocate a `b3Sphere` with center (0,0,0) and radius 0.075.
   - Shape density so the mass is 0.215 kg: `0.215 / (4/3 π r³)` ≈ 121.6 kg/m³.
   - Restitution 0 for this test. Set `b3SurfaceMaterial.restitution` inside `b3ShapeDef.baseMaterial`.
   - `Box3D.b3CreateSphereShape(...)`.
5. Step 120 times with `Box3D.b3World_Step(world, 1f/60f, 4)`.
6. `pos = Box3D.b3Body_GetPosition(arena, ball)`. Assert `pos.z` is within 0.075 ± 0.01 and |x|, |y| < 1e-3.
7. `Box3D.b3DestroyWorld(world)`.

The test JVM already gets `--enable-native-access=ALL-UNNAMED` (step 01), so there is no warning noise.

## Gotchas

- Every `b3XxxDef` has an `internalValue` set by `b3DefaultXxxDef()` and checked by asserts. Always start from the
  default function. Never `allocate()` a definition and fill it from zero.
- Event arrays (`b3SensorEvents`, `b3ContactEvents`, `b3BodyEvents`) point into Box3D memory that is valid only until
  the next step. Copy what you need right away. Reinterpret the pointer with
  `ptr.reinterpret(count * layout.byteSize())`.
- No upcalls in v0. Upcalls are friction/restitution callbacks, custom filters, pre-solve and ray callbacks. An
  exception thrown in an upcall kills the JVM, and several upcalls run on Box3D worker threads.

## Acceptance

- `pwsh scripts/get-jextract.ps1` and then `pwsh scripts/generate-bindings.ps1` produce `src/main/java/box3dsim/ffm/gen/`.
- `.\gradlew.bat build` passes, and `Box3dSmokeTest` passes.
- `git grep -n 'findOrThrow("b3MakeQuat")\|findOrThrow("b3Add")' src/main/java/box3dsim/ffm/gen` finds nothing,
  because inline helpers are excluded. A plain `b3MakeQuat` grep matches the exported `b3MakeQuatFromMatrix`.
- Add `-Xlint:-restricted` to the compiler args in `build.gradle`. The generated code calls restricted FFM methods
  everywhere, and `-Xlint:all` would print hundreds of warnings.

Commit: `Generate Box3D FFM bindings and native loader`.
