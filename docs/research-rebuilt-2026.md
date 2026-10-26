# 2026 REBUILT: rules and field facts used by the sim

Researched 2026-10-26. All positions are in the WPILib blue-origin field frame: +x toward red, +y left from
blue, and the scoring-table side at y ≈ 0. Red elements are blue ones rotated 180° about the field center.

## Sources

| Tag | Source |
|---|---|
| [M] | Game Manual, TU22: https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf |
| [D] | Field dimension drawings FE-2026 Rev B: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf |
| [T] | WPILib `2026-rebuilt-welded.json` and `-andymark.json`. 2027 moved these to `fields/src/main/native/resources/org/wpilib/fields/frc/` |
| [FC] | Fuel Counter Assessment, 2026-03-02 |
| [AS] | AdvantageScope assets `default-assets-v2` (`Field3d_2026FRCFieldV2`) |

## Field

- Welded field: 16.541 × 8.069 m [T]. AndyMark: 16.518 × 8.043 m.
- The AdvantageScope model matches welded.
- Guardrails are 20 in (0.508 m) tall.
- ALLIANCE ZONE depth: 158.6 in (the ROBOT STARTING LINE at x = 4.028 m on blue). CENTER LINE: x = 8.270 m.

## FUEL

- 5.91 in (15.0 cm) foam ball, 0.448–0.500 lb (0.203–0.227 kg) [M 5.10.1].
- 504 at match start [M 6.3.4]:
  - 24 in each depot
  - 24 in each outpost chute
  - up to 8 preloaded per robot
  - 360–408 in the neutral zone
- [AS] stages 456 FUEL:
  - **Neutral zone:** 12 × 30 at 6 in pitch, 12 columns at x = L/2 + (i − 5.5)·0.1524, 15 rows each side of
    y = W/2 at ±(0.1016 + k·0.1524), with a 2 in center gap.
  - **Blue depot:** a 4 × 6 grid at x ∈ {0.0762 … 0.5334}, y ∈ {5.5839 … 6.3459}.
  - **Blue chute:** 24 balls on the 15° slope behind the wall near y = 0.36–0.97, z = 0.81–0.96.
- [AS] hides its staged FUEL as soon as any FUEL poses are published.

## HUB

- 47 in square.
- Blue center (4.6255, 4.0346), red center (11.9155, 4.0346) [T].
- Hex opening 41.7 in inside, front edge at 72 in (1.83 m) [M 5.4; D]. An upper back structure (net, inferred)
  58.41 in wide reaches 120.36 in.
- Four exits at the base face the NEUTRAL ZONE; output is random. Exit geometry and speeds are unpublished.
- FUEL counts after passing the sensor array [M 6.5.1]: mean 1.32 s after entry, always under 3 s [FC].
  Assessment continues 3 s after a HUB deactivates and 3 s after AUTO or TELEOP ends [M 6.5]. The lights pulse
  3 s before deactivation.

## Match timeline and HUB status [M 6.4, Tables 6-2, 6-3]

| Period | Duration | Active HUBs |
|---|---|---|
| AUTO | 20 s | both |
| (3 s delay before TELEOP, for scoring) | | |
| TRANSITION | 10 s | both |
| SHIFT 1–4 | 25 s each | one, alternating |
| END GAME | 30 s | both |

- TELEOP is 2:20 in total.
- The alliance with more AUTO FUEL goes **inactive in SHIFT 1** (and SHIFT 3). Ties are random.
- Game data `"R"`/`"B"` names the first-inactive alliance and arrives about 3 s after AUTO.

## Points [M Table 6-4]

- FUEL in an active HUB: 1 point. In an inactive HUB: 0.
- TOWER: AUTO L1 15 (max 2 robots); TELEOP L1 10, L2 20, L3 30.
- Fouls: MINOR 5, MAJOR 15.
- Ranking points (regional): ENERGIZED ≥ 100 active FUEL, SUPERCHARGED ≥ 360, TRAVERSAL ≥ 50 tower points.

## Rules the sim enforces or notes

| Rule | Meaning | Sim |
|---|---|---|
| G407 | may not launch into **your own** HUB unless your BUMPERS are partly in your ALLIANCE ZONE (MAJOR). Launching elsewhere, such as passing from midfield, is legal | enforced: only when an out-of-zone launch enters the launcher's own HUB |
| G403 | no fully crossing the CENTER LINE in AUTO | not enforced |
| G405 | no ejecting FUEL off the field except via the OUTPOST base opening | out-of-field count only |
| G425 / G427 | human player FUEL entry routes; OUTPOST storage limit | modeled as chute/corral actions |
| G303 | start overlapping the line, not on a BUMP, ≤ 8 FUEL | preload 8 |

## Field elements [M 5.5–5.9, D]

| Element | Facts |
|---|---|
| BUMP | 73.0 in (y) × 44.4 in (x) × 6.513 in, HDPE 15° ramps (the manual's numbers don't close geometrically). Either side of the HUB: y [1.583, 3.438] and [4.631, 6.486] m |
| TRENCH | structure 65.65 × 47.0 × 40.25 in. Opening 50.34 in wide × **22.25 in** clearance, guardrail to BUMP, with a 12 in post next to the BUMP |
| TOWER | 49.25 × 45.0 × 78.25 in, in the alliance wall. Center y 3.746 m (blue). Uprights 72.1 in, 32.25 in apart. Rungs 1.66 in OD at 27/45/63 in |
| DEPOT | 42 × 27 in against the alliance wall. Blue center y 5.963 m. Barrier strips 1 in tall |
| OUTPOST | at the scoring-table corner, blue centerline y 0.666 m. CHUTE: 15° tunnel of about 25 FUEL with a door; field opening 31.8 × 7 in at 28.1 in. CORRAL base opening 32 × 7 in at 1.88 in, split by a 1.66 in pipe; robots push FUEL in; corral 35.8 × 37.6 in |

## Robot limits [M 8.1, 8.4]

- Frame perimeter ≤ 110 in. Height ≤ 30 in (≤ 22.25 in to fit under a TRENCH).
- BUMPER ZONE 2.5–5.75 in.
- Weight ≤ 115 lb without bumpers and battery.

## AdvantageScope

- 3D field "2026 Field" uses `wall-blue` coordinates (same as WPILib). Its one game piece type is "Fuel".
- Robot: built-in "2026 KitBot". Components are robot-relative `Pose3d[]` matched to `model_N.glb` through a custom
  robot `config.json` (`components: [{zeroedRotations, zeroedPosition}]`).
- Live: File → Connect to Simulator. Tuning mode can write only `/Tuning/...` keys.
- Layouts: File → Export/Import Layout. The JSON is tied to the AdvantageScope major version.
