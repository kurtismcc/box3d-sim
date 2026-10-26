# Step 09: 2026 REBUILT match rules, HUB scoring and OUTPOST

**Goal:** a participant that runs the REBUILT match clock and HUB active/inactive shifts, scores FUEL that enters a
HUB, returns it to the field through the HUB exits, and runs both OUTPOSTs (CORRAL intake, CHUTE release,
human-player throws).

**Depends on:** step 08.

Package `box3dsim.season.rebuilt2026`.

## Why the sim owns the match clock

The WPILib 2027 sim GUI doesn't run a timed match. It has no auto→teleop transition and no disable at the end, and
match time stays −1 unless typed in. The GUI also re-pushes its DS state every frame, so the sim can't drive the
DS from code. The rules engine therefore runs its own clock from the enable/mode edges it observes, and it tells
the robot integration (step 11) when robot outputs must be cut, the way the FMS would disable the robot.

## Inputs (set every robot loop by step 11, before the substeps)

```java
public record DsState(boolean enabled, boolean autonomous, boolean teleop) {}
```

## Match state machine

```java
public enum MatchPhase { FREE_PLAY, AUTO, AUTO_TO_TELEOP, TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, END_GAME, POST_MATCH }
```

The names match the robot's own `States.GameMode` where they overlap.

| From | Event | To |
|---|---|---|
| FREE_PLAY | `enabled && autonomous` rising edge | AUTO, `autoStart = now` |
| FREE_PLAY | `enabled && teleop` rising edge | TRANSITION (a teleop-only practice match), `teleopStart = now` |
| AUTO | `now − autoStart ≥ 20` | AUTO_TO_TELEOP. Pick the first-inactive alliance (below) once auto scoring has settled, 3 s after auto ends [M 6.5] |
| AUTO_TO_TELEOP | `enabled && teleop` rising edge (require that DS was seen disabled or in auto since) | TRANSITION, `teleopStart = now` |
| TRANSITION … END_GAME | from `t = now − teleopStart`: [0,10) TRANSITION, [10,35) SHIFT_1, [35,60) SHIFT_2, [60,85) SHIFT_3, [85,110) SHIFT_4, [110,140) END_GAME | next phase |
| END_GAME | `t ≥ 140` | POST_MATCH |
| any but FREE_PLAY | `resetMatch()` | FREE_PLAY |

- **Robot outputs allowed:** `outputsAllowed()` is false in AUTO_TO_TELEOP and POST_MATCH, true otherwise. Step 11
  disables every `SparkSim` while it's false. The DS disabling the robot is separate and handled by REVLib.
- **Match time** for display: AUTO counts down `20 − (now − autoStart)`; teleop counts down `140 − t`; FREE_PLAY
  and POST_MATCH show −1.

## HUB active status [M 6.4.1]

- Both HUBs are active in FREE_PLAY, AUTO, AUTO_TO_TELEOP, TRANSITION and END_GAME.
- **First inactive alliance:** the alliance with more FUEL scored in AUTO goes inactive in SHIFT_1 and SHIFT_3 and is
  active in SHIFT_2 and SHIFT_4. The other alliance gets the opposite. On a tie, pick with `new Random(seed)`
  (seed in config). In a teleop-only practice match, pick at random at teleop start.
- `firstInactive()` returns `Optional<Alliance>`. It is empty until picked. This mirrors the "R"/"B" game data
  [WPILib 2026 game data page].
- `deactivatedAt(alliance)`: the time the alliance's HUB last went from active to inactive (or −∞).

## HUB scoring

For each alliance's `hubSensors[a]` begin event in `postPhysics`:

1. Look up the FUEL. Ignore visitors that aren't FUEL.
2. Remove the FUEL from the world (`GamePieces.remove`) and queue a `PendingFuel(a, entryTime = now,
   countTime = now + U(0.8, 1.8), rng)`. The processing time from funnel entry to count averaged 1.32 s, all under
   3 s [FC 2026-03-02]. U(0.8, 1.8) is an ASSUMPTION matching that mean.
3. When `now ≥ countTime`, **count** it:
   - Increment `fuelProcessed[a]`.
   - Award 1 point to alliance `a` if the HUB counts as active, meaning it is active at `countTime`, or
     `countTime − deactivatedAt(a) ≤ 3.0` (FUEL keeps being assessed 3 s after deactivation [M 6.5]).
   - It also needs a **scoring window**: the phase is AUTO, or AUTO_TO_TELEOP with `now − autoEnd ≤ 3`, or
     TRANSITION through END_GAME, or POST_MATCH with `now − teleopEnd ≤ 3`.
   - In FREE_PLAY, count it into `freePlayFuel[a]` and award no points.
   - Track `autoFuel[a]` for points counted during the AUTO window. It decides `firstInactive`.
   - **Return** the FUEL through a random exit (below).
4. Points: FUEL is 1 point in AUTO or TELEOP when active, 0 when inactive [M Table 6-4]. Towers are not modeled
   (the 2026-Offseason robot has no climber), so tower points are always 0.

### HUB exits (FUEL returned to the field)

4 exits at the HUB base on the NEUTRAL ZONE face [M 5.4]. For blue (mirror for red):

- Exit position: x = 5.2225 + 0.10 and z = 0.12, with y = 4.0346 + {−0.4477, −0.1492, +0.1492, +0.4477}.
- Spawn velocity: speed U(1.0, 3.0) m/s, yaw {−33.75°, −11.25°, +11.25°, +33.75°} (one per exit) + U(−10°, +10°),
  pitch 0, no spin.
- Choose the exit uniformly at random.

The exit geometry and speeds are ASSUMPTIONS taken from maple-sim's values. FIRST says the output is "randomly"
distributed.

## OUTPOST (one per alliance)

State: `chute` (stored count, starts 24 [M 6.3.4]) and `corral` (starts 0). Capacities: chute 25 [M 5.9.2], corral
30 (ASSUMPTION).

- **CORRAL intake:** on a `corralSensors[a]` begin event with a FUEL visitor, remove the FUEL and `corral++`. If the
  corral is full, the human player must enter excess FUEL immediately (G427), so auto-trigger one
  `releaseChute` FUEL instead (below).
- **Human-player actions.** Public methods, triggered by step 11 from sim-only controls:
  - `refillChute(a)`: move `min(corral, 25 − chute)` from corral to chute. Instant.
  - `releaseChute(a)`: open the CHUTE DOOR. Release one FUEL every 0.15 s until the chute is empty, or until
    `closeChute(a)`.
    - Spawn position (blue): x = 0.10, y = 0.666 + U(−0.30, 0.30), z = 0.80 (the field-side CHUTE opening is
      31.8 in wide, bottom 28.1 in [M 5.9.2]).
    - Velocity: (U(1.0, 2.0), U(−0.2, 0.2), −0.3) m/s, rotated for red (mirror the position; negate vx and vy).
  - `closeChute(a)`.
  - `throwAtHub(a)`: if `chute + corral > 0`, take one FUEL (corral first) and throw it over the alliance wall at
    that alliance's HUB (G425 allows throwing from the OUTPOST AREA).
    - Release point (blue): (−0.40, 1.20, 1.70).
    - Aim at `hubCenter(a) + (0, 0, 0.30)`. Use a fixed pitch of 55°, and solve the vacuum speed
      `v = sqrt(g·d² / (2·cos²θ·(d·tanθ − Δh)))`. Multiply it by 1.08 to make up for drag (ASSUMPTION).
    - Then add human noise: speed ×N(1, 0.04), yaw +N(0, 2°), pitch +N(0, 2°).
    - Backspin 0.
- **Display:** `fuelDisplayPoses()` returns every live FUEL body pose, plus `chuteDisplay(a)` for the first `chute`
  slots and `corralDisplay(a)` for the first `corral` slots of each alliance. Rotations are identity. AdvantageScope
  hides its staged FUEL once poses are published, so this must include stored FUEL.

## Fouls (cheap subset)

`onLaunch(Alliance a, Pose3 robotPose, double hx, double hy)` is called by the shooter (step 10) for every launched
FUEL. `hx` and `hy` are the bumper half-length and half-width.

- **G407:** the robot's BUMPERS must be partly in its own ALLIANCE ZONE. For blue, foul if the bumper's nearest
  x extent is past the line: `robotPose.x − (|cos ψ|·hx + |sin ψ|·hy) > STARTING_LINE_X`, where ψ is the robot
  yaw. Red: foul if `robotPose.x + (|cos ψ|·hx + |sin ψ|·hy) < L − STARTING_LINE_X`.
- **Penalty:** MAJOR FOUL, 15 points to the opponent [M 6.6]. Only during the match (not FREE_PLAY).
- Count `fouls[a]` and log the last foul reason as a string.

Other fouls are out of scope for v0.

## Public API

```java
public final class RebuiltMatch implements ArenaParticipant {
    public record Config(long seed, boolean spawnNeutralZone, boolean spawnDepots, int robotPreload) {
        public static Config defaults();   // 4470, true, true, 8
    }
    public RebuiltMatch(Arena arena, RebuiltFieldHandles field, Config config);
    public void setDsState(DsState ds);
    public void resetMatch();               // phase FREE_PLAY, scores 0, outposts 24/0, respawn field FUEL (asleep)
    public MatchPhase phase();
    public double matchTimeRemaining();     // −1 when not in a timed period
    public double phaseTimeRemaining();
    public boolean outputsAllowed();
    public boolean hubActive(Alliance a);
    public Optional<Alliance> firstInactive();
    public int score(Alliance a);
    public int autoFuel(Alliance a); public int fuelProcessed(Alliance a); public int fouls(Alliance a);
    public int freePlayFuel(Alliance a);
    public int chuteCount(Alliance a); public int corralCount(Alliance a);
    public void refillChute(Alliance a); public void releaseChute(Alliance a); public void closeChute(Alliance a);
    public void throwAtHub(Alliance a);
    public void onLaunch(Alliance a, Pose3 robotPose, double hx, double hy);
    public List<Pose3> fuelDisplayPoses();
    public String lastFoul();
}
```

`resetMatch()` first calls `arena.gamePieces().clear()` and clears pending HUB FUEL. It then spawns the neutral zone
and both depots with `awake = false` (`GamePieces.reset` deliberately doesn't clear; see step 07). `reset(arena)`
from `ArenaParticipant` calls `resetMatch()`. Robots' preloads are handled by the robot's storage (step 10) through
its own `reset`.

The constructor also calls `arena.gamePieces().setFieldBounds(0, RebuiltField.LENGTH, 0, RebuiltField.WIDTH)`.
FUEL in the CORRAL pocket (x down to −0.955) stays within the 1.0 m out-of-field margin.

**Time source:** `arena.time()`. All randomness comes from one `Random(config.seed)`.

## Tests: `src/test/java/box3dsim/season/rebuilt2026/RebuiltMatchTest.java`

Arena + `RebuiltField.build` + `RebuiltMatch`. Drive the DS state by hand and use `arena.advance(seconds)`.

1. **Timeline:** enable auto at t = 0. At t = 19.9 the phase is AUTO; at 20.1 it is AUTO_TO_TELEOP and
   `outputsAllowed()` is false. Disable, then enable teleop at t = 25. Phases are TRANSITION at 25 + 5, SHIFT_1 at
   25 + 11, SHIFT_4 at 25 + 100, END_GAME at 25 + 111, and POST_MATCH at 25 + 141.
2. **Shift alternation:** force `autoFuel(BLUE) = 3, autoFuel(RED) = 1` by dropping FUEL into the HUBs during auto.
   Then BLUE is inactive in SHIFT_1 and SHIFT_3 and active in SHIFT_2 and SHIFT_4; RED is the opposite; both are
   active in TRANSITION and END_GAME.
3. **Scoring delay and return:** in TRANSITION, drop one FUEL from z = 2.5 into the blue HUB. Times are from the
   drop; the fall takes about 0.4 s and counting adds 0.8–1.8 s. The blue score is unchanged at +0.5 s and has
   risen by 1 at +2.5 s. The FUEL count on the field is restored by then (the returned FUEL appears with x > 5.2).
4. **Inactive grace:** drop a FUEL into BLUE's HUB 1.0 s before BLUE goes inactive. It still scores, because it is
   counted ≤ 3 s after deactivation. Drop one 4.0 s after deactivation: it scores 0 but is still returned.
5. **Free play:** in FREE_PLAY a FUEL in the HUB increments `freePlayFuel`, not the score, and is returned.
6. **Corral and chute:** roll a FUEL into the blue CORRAL, and `corralCount(BLUE)` becomes 1. `refillChute` moves it
   to the chute (25 cap). `releaseChute` empties a 25-chute in about 3.75 s, spawning FUEL near x = 0.1.
7. **Human throw:** with drag on, 20 `throwAtHub(BLUE)` calls in TRANSITION score at least 6 (`fuelProcessed`).
   This checks that the aim solver points the right way. The threshold is loose by design.
8. **G407:** `onLaunch(BLUE, pose x = 6.0)` during TRANSITION adds 15 to RED and increments `fouls(BLUE)`;
   `onLaunch(BLUE, pose x = 3.0)` adds nothing.
9. **Display:** after reset, `fuelDisplayPoses().size() == 360 + 48 + 48`. That is AdvantageScope's staged count
   (456), and it proves stored FUEL is included.

## Acceptance

- `.\gradlew.bat test` passes.

Commit: `Add REBUILT match rules, hub scoring and outposts`.
