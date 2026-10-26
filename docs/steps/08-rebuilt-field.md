# Step 08: 2026 REBUILT field geometry and starting fuel

**Goal:** a static Box3D model of the 2026 REBUILT welded field that robots and FUEL interact with, the FUEL game
piece type, and the exact starting FUEL layout.

**Depends on:** step 07.

Package `box3dsim.season.rebuilt2026`. Also add `box3dsim.arena.Alliance { BLUE, RED; Alliance opponent(); }`.

## Sources

| Tag | Source |
|---|---|
| [M] | 2026 Game Manual TU22 |
| [D] | Field dimension drawings FE-2026 Rev B |
| [T] | WPILib `2026-rebuilt-welded` AprilTag layout |
| [AS] | AdvantageScope `Field3d_2026FRCFieldV2` model. Its staged FUEL positions were extracted for this doc; AdvantageScope hides its own staged FUEL once the sim publishes FUEL poses, so the sim must publish every ball |

Items marked **ASSUMPTION** are not given by any source. Keep each one a named constant so it can be tuned.

## Frame and mirroring

- WPILib blue-origin field frame: +x toward red, +y left from blue, z up.
- Field length `L = 16.541` m (651.22 in) and width `W = 8.069` m (317.677 in) [T, AS].
- The scoring-table side is y ≈ 0.
- The red half is the blue half rotated 180° about the field center: `mirror(x, y, z) = (L − x, W − y, z)`, and
  `yaw → yaw + π`.
- Build every element below once for blue and once through `mirror` for red.

## Materials (`RebuiltMaterials`)

| Name | friction | restitution | rolling resistance | Use |
|---|---|---|---|---|
| `CARPET` | 0.8 | 0.0 | 0.06 | floor. ASSUMPTION |
| `POLYCARB` | 0.4 | 0.3 | 0 | guardrails, walls, hub, trench, tower |
| `HDPE` | 0.3 | 0.2 | 0 | BUMP ramps |
| `FUEL` | 0.6 | 0.35 | 0.05 | game piece. ASSUMPTION; calibrate in step 12 |

Box3D mixes restitution as the max and friction as the geometric mean. The carpet restitution is therefore 0, so
the ball's own value sets the bounce.

## FUEL game piece type

`RebuiltFuel.TYPE = new GamePieceType("Fuel", 0.075, 0.215, FUEL, 0.50, 0.6, 0.30, 0.5, 0.0, 0.0)`.

- Diameter 5.91 in (15.0 cm) and mass 0.203–0.227 kg come from [M 5.10.1]; 0.215 kg is the midpoint.
- C_d 0.5, liftSlope 0.6, liftMax 0.3 and spin decay 0.5/s are ASSUMPTIONS (typical foam sphere values).

## Static elements (blue side; mirror for red)

All boxes are given as **min/max corners** in meters. Convert each to `ShapeSpec.Box(halfExtents, localPose)` on one
static body per element group.

| Element | Geometry | Layer, material |
|---|---|---|
| Carpet | box x [−1, L+1], y [−1, W+1], z [−0.1, 0] | FIELD, CARPET |
| Guardrail, scoring side | box x [0, L], y [−0.1, 0], z [0, 0.508] (20 in [M 5.2]) | FIELD, POLYCARB |
| Guardrail, far side | box x [0, L], y [W, W+0.1], z [0, 0.508] | FIELD, POLYCARB |
| Blue alliance wall | box x [−0.1, 0], z [0, 2.0], **except** the CORRAL opening: build as 3 boxes: y [−0.1, 0.2596], y [1.0724, W+0.1], and above the opening y [0.2596, 1.0724] z [0.2256, 2.0]. Plus a sill y [0.2596, 1.0724] z [0, 0.0478] | FIELD, POLYCARB |
| CORRAL pocket | static floor is the carpet (it extends to x = −1). Side walls: boxes x [−0.955, −0.1], y [0.2096, 0.2596] and [1.0724, 1.1224], z [0, 0.2065]. Back wall: x [−1.0, −0.955], y [0.2096, 1.1224], z [0, 0.2065]. The CORRAL is 35.8 × 37.6 in with 8.13 in panels [M 5.9.2] | FIELD, POLYCARB |
| CORRAL divider pipe | vertical box x [−0.05, 0], y [0.645, 0.687], z [0, 0.2256] (stands in for the 1.66 in OD pipe [M]) | FIELD, POLYCARB |
| CORRAL sensor | box x [−0.9, −0.15], y [0.27, 1.06], z [0, 0.20] | SENSOR, sensor |
| HUB base | box x [4.0285, 5.2225], y [3.4376, 4.6316], z [0, 1.40]. 47 in square centered (4.6255, 4.0346) [T]. Top height 1.40 is an ASSUMPTION (internal funnel unknown) | FIELD, POLYCARB |
| HUB rim | 4 boxes, 0.064 m thick, z [1.40, 1.829] (72 in [M 5.4]) along the 4 edges of the base. Inner opening ≈ 1.066 m ≈ the 41.7 in hex [D]. Square rim is a simplification of the hex | FIELD, POLYCARB |
| HUB score sensor | box x [4.13, 5.12], y [3.54, 4.53], z [1.42, 1.70] | SENSOR, sensor |
| HUB back net | box x [5.2225, 5.2425], y [3.2926, 4.7766] (58.41 in), z [1.829, 3.057] (120.36 in) [D sh4]. On the NEUTRAL ZONE face. ASSUMPTION that it's a solid panel | FIELD, POLYCARB |
| BUMPs (2) | convex hull: a triangular prism with base x [4.0615, 5.1895] (44.4 in), ridge along y at x = 4.6255, height 0.1654 (6.513 in) [M 5.5]. y ranges [1.5834, 3.4376] and [4.6316, 6.4856] (73.0 in). The manual's 15° ramps don't reach 6.513 in over 44.4 in; this prism is ≈16.3°. Hull points: 4 base corners plus 2 ridge ends | **PIECE_BLOCKER**, HDPE |
| TRENCH arm, scoring side | box x [4.0285, 5.2225] (47 in deep), y [0, 1.2786] (50.34 in opening), z [0.5652, 1.0224] (22.25 in clearance, 40.25 in tall) [M 5.6] | FIELD, POLYCARB |
| TRENCH post, scoring side | box x [4.0285, 5.2225], y [1.2786, 1.5834] (12 in), z [0, 1.0224] | FIELD, POLYCARB |
| TRENCH arm and post, far side | same, with y reflected about W within the blue half: arm y [W−1.2786, W], post y [W−1.5834, W−1.2786] | FIELD, POLYCARB |
| TOWER uprights (2) | box x [1.0175, 1.1065] (3.5 in), y [3.2962, 3.3342] and [4.1535, 4.1915] (1.5 in, centers 3.3152 / 4.1725 = 33.75 in c-c), z [0, 1.831] (72.1 in) [M 5.8; maple-sim positions] | FIELD, POLYCARB |
| TOWER rungs (3) | `Capsule` from (1.062, 3.1660, z) to (1.062, 4.3217, z), radius 0.0211 (1.66 in OD, 5.875 in past each upright), z = 0.6858, 1.1430, 1.6002 (27/45/63 in) | FIELD, POLYCARB |
| TOWER support | box x [0, 1.0175], y [3.3342, 4.1535], z [0.7214, 1.1019] (28.40–43.38 in). ASSUMPTION about its footprint | FIELD, POLYCARB |
| DEPOT barriers | boxes 0.0254 m tall, 0.0762 m wide: along x = 0.6858 (y [5.4296, 6.4964]), and along y = 5.4296 and y = 6.4964 (x [0, 0.6858]). Centered on the edges. DEPOT is 42 × 27 in, centered y = 5.963 [M 5.7, D] | FIELD, POLYCARB |

The rungs use `ShapeSpec.Capsule` from step 04.

Robots are not blocked by BUMPs in v0. Their chassis is planar (step 06) and can't ride a ramp, so BUMPs are
`PIECE_BLOCKER`, and robots drive across them as if the floor were flat. Balls still roll over and bounce off them.
List this in step 12's known-limitations.

## Field handles

```java
public record RebuiltFieldHandles(Map<Alliance, ShapeHandle> hubSensors,
                                  Map<Alliance, ShapeHandle> corralSensors,
                                  List<BodyHandle> staticBodies) {}

public final class RebuiltField {
    public static final double LENGTH = 16.541, WIDTH = 8.069;
    public static final Vec3 BLUE_HUB_CENTER = new Vec3(4.6255, 4.0346, 1.829);   // top-opening center
    public static final double STARTING_LINE_X = 4.0284;    // 158.6 in; blue ALLIANCE ZONE is x ≤ this
    public static final double CENTER_LINE_X = LENGTH / 2;
    public static Vec3 mirror(Vec3 p);
    public static Vec3 hubCenter(Alliance a);
    public static boolean inAllianceZone(Alliance a, double x);   // blue: x ≤ 4.0284; red: x ≥ L − 4.0284
    public static RebuiltFieldHandles build(SimWorld world);
}
```

## Starting FUEL layout (`RebuiltFuelLayout`)

Positions are FUEL centers. z = 0.075 for all floor FUEL. They are exact from [AS] and match [M 6.3.4] counts.

```java
public static List<Vec3> neutralZone();               // 360
public static List<Vec3> depot(Alliance a);           // 24
public static List<Vec3> chuteDisplay(Alliance a);    // 24 display slots, back to front (see below)
public static List<Vec3> corralDisplay(Alliance a);   // 30 display slots
```

- **Neutral zone (360):**
  - `x_i = L/2 + (i − 5.5)·0.1524` for i = 0..11 (12 columns at 6 in pitch, ±0.8382).
  - `y = W/2 ± (0.1016 + k·0.1524)` for k = 0..14 (15 rows each side of center). The 2 in center divider leaves a
    0.2032 m gap [M 6.3.4.1].
  - 12 × 30 = 360.
- **Blue depot (24):** x ∈ {0.0762, 0.2286, 0.3810, 0.5334} and y = 5.5839 + j·0.1524 for j = 0..5 (4 × 6 grid).
  Red = `mirror`.
- **Blue chute (display only, 24):** these balls are not physics bodies; they are a count held by the OUTPOST (step 09).
  - Top row: x = −0.6389, z = 0.9616, y = 0.4372 + j·0.1524 for j = 0..3.
  - Then 4 rows of 5: x = −0.5114, −0.3642, −0.2170, −0.0698 with z = 0.9274, 0.8880, 0.8485, 0.8091, and
    y = 0.3610 + j·0.1524 for j = 0..4.
  - List them back to front (top row first). With `n` FUEL stored, display the **first** `n`, so the front empties first.
- **Blue corral (display only, 30):** a 5 × 6 grid at x = −0.85 + c·0.16 (c = 0..4), y = 0.30 + r·0.14 (r = 0..5),
  z = 0.075. Display the first `n`. ASSUMPTION (visual only).

## Tests: `src/test/java/box3dsim/season/rebuilt2026/RebuiltFieldTest.java`

Arena with `RebuiltField.build(world)`. FUEL spawned with `RebuiltFuel.TYPE`.

1. **Counts:** `neutralZone().size() == 360`, `depot(BLUE).size() == 24`, `chuteDisplay(RED).size() == 24`. All
   neutral-zone and depot positions are within the field and at least 0.15 m apart.
2. **Mirror:** `mirror(mirror(p)) == p`, and `hubCenter(RED).x == L − 4.6255`.
3. **Hub catch:** a FUEL dropped from (4.6255, 4.0346, 2.5) produces a sensor begin event on `hubSensors[BLUE]`
   within 1 s, and comes to rest on the HUB base top (z ≈ 1.475).
4. **Hub rim:** a FUEL dropped onto the rim edge at (4.0285 + 0.032, 4.0346, 2.5) does not rest inside the sensor
   volume after 2 s (it bounces off, in or out).
5. **Trench clearance:** a FUEL rolled at 2 m/s along +x at y = 0.6, from x = 3.0, passes under the arm
   (x > 5.3 after 2 s).
6. **Bump:** a FUEL rolled at 3 m/s along +x at y = 2.5 from x = 3.5 rises above z = 0.2 while crossing the BUMP
   ridge. A box-shaped planar `ROBOT` body (0.9 × 0.9 × 0.3, `gravityScale 0`, `MotionLocks.PLANAR`) given
   vx = 2 m/s along the same line is not stopped: its x passes 5.3 within 1.5 s.
7. **Corral:** a FUEL rolled at 2.5 m/s along −x from (1.0, 0.45, 0.075) enters `corralSensors[BLUE]` within 2 s.
   y = 0.45 avoids the divider pipe at y ≈ 0.666, and 2.5 m/s clears the 0.048 m sill.
8. **Field bounds:** a FUEL rolled at 4 m/s toward the scoring-side guardrail bounces back (y stays ≥ 0.075 − 0.01).
9. **Settled start:** spawn all 360 + 48 FUEL asleep. After 1 s, no FUEL has moved more than 1 mm, and the awake count is 0.

## Acceptance

- `.\gradlew.bat test` passes.

Commit: `Add 2026 REBUILT field geometry and starting fuel layout`.
