# box3d-sim v0 plan

## Goal

Launch the retired 2026-Offseason robot (`D:\Projects\2026-Offseason`, SoCal Showdown, 2026 REBUILT) in desktop
simulation, watch it in AdvantageScope, and play a full match solo. Underneath is a real 3D world: FUEL bodies
bounce off HUB rims, pile up, get pushed by bumpers, and get sucked up and shot by mechanisms driven by the robot's
own motor controllers.

v0 is done when:

1. `.\gradlew.bat run` in the robot repo starts the sim GUI, and the robot drives with a gamepad. Odometry tracks
   the true pose.
2. The intake deploys and collects FUEL. The shooter spins up and launches real FUEL bodies, and the flywheel sags
   per shot.
3. A full match runs: 20 s AUTO, then TELEOP with TRANSITION, four alternating HUB shifts and END GAME. Scoring
   follows the active/inactive rules, and scored FUEL returns to the field through the HUB exits.
4. AdvantageScope's 3D field shows the robot, every FUEL, the score and the HUB states live.
5. The `Shoot at Hub` PathPlanner auto runs and scores.
6. `.\gradlew.bat run -PsimMode=fast -PsimAuto="Shoot at Hub"` plays the auto headless, faster than real time, and
   exits 0 or 1 with a results file. That makes the sim usable from a CI script.

## Inputs

| Doc | What it holds |
|---|---|
| [robot-2026-offseason.md](robot-2026-offseason.md) | everything the sim must model about the robot (motors, configs, controls, autos, sim pitfalls) |
| [maple-sim-research.md](maple-sim-research.md) | what maple-sim models, and what we keep vs replace |
| [research-box3d.md](research-box3d.md) | Box3D status, C API, build, Java FFM notes |
| [research-2027-sim-apis.md](research-2027-sim-apis.md) | WPILib 2027 alpha 7, REVLib, Phoenix 6, AdvantageKit, PathPlanner and Limelight sim behavior |
| [research-rebuilt-2026.md](research-rebuilt-2026.md) | game rules, field geometry, FUEL facts, AdvantageScope field |

## Architecture

```
D:\Projects\2026-Offseason (robot repo, branch box3d-sim)
  subsystems / commands / RobotContainer            unchanged robot logic
  REV SparkMax / SparkFlex, Pigeon2                 real vendor classes
  frc.robot.sim  (only package importing box3dsim)
    RobotSim        per 20 ms: 5 × { SparkSim.iterate → volts → arena.step() → write encoders }
    SparkBinding    REV SparkSim ↔ one simulated DOF (encoder offsets, turn-angle wrapping)
    SimConstants    robot geometry, gearing, mechanism assumptions
    logging         AdvantageKit Logger.recordOutput → NT4 → AdvantageScope
        │  volts in / positions, velocities, yaw, battery out
F:\Projects\box3d-sim (library, box3dsim:box3d-sim, no WPILib dependency)
  season.rebuilt2026   field geometry, FUEL layout, match clock, HUB scoring, OUTPOST
  mechanism            IntakeSim, FuelStorage, ShooterSim
  drive                SwerveDriveSim (tire slip, load transfer), GyroSim
  gamepiece            3D FUEL bodies, drag, Magnus, out-of-field
  motor                DcMotor, Battery, LumpedMechanism
  arena                Arena step loop, participants, electrical loads
  world                SimWorld interface ── Box3dWorld backend
  ffm                  jextract bindings + native loader
  natives/windows-x86_64/box3d.dll   (Box3D @ f36d061)
```

### Decisions

1. **REV's own `SparkSim` runs every Spark closed loop.** REVLib 2027 alpha 8 ports the firmware controllers
   (PID, kV, MAXMotion, smart current limit) to native sim code. Using it keeps the robot's `Configs.java` tuning
   meaningful without re-implementing REV control in Java. The library only sees volts.
2. **The library has no WPILib dependency.** It has its own `Vec3`, `Quat` and `Pose3`, plus its own motor
   constants. The 2027 alphas churn, and the library's tests run as plain JUnit. The robot repo converts to WPILib
   types.
3. **Box3D through jextract-generated FFM bindings** on a pinned commit, hidden behind `SimWorld`. Box3D is alpha
   and its API moved after `v0.1.0`. The interface leaves room for a Jolt (jolt-jni) backend if needed.
4. **The chassis is a planar dynamic body** (z, roll and pitch locked, no gravity) and never touches the carpet.
   Traction comes entirely from a per-wheel slip-velocity tire model with a friction circle and analytic load
   transfer. That's more accurate than maple-sim's cap-and-blend and avoids fighting the contact solver. The cost:
   BUMPs can't be driven over physically, so robots ignore them in v0. (Revised in step 14: with a
   `SuspensionConfig` the chassis is free in six DOF and finds the ground by ray casts; the same tire model acts in
   the contact plane with normal loads from per-wheel springs. BUMPs collide with robots.)
5. **FUEL are always real bodies.** There is no analytic projectile. Air drag and Magnus lift are applied per
   step, and sleeping keeps the ~400 resting FUEL cheap.
6. **The sim owns the match clock.** The 2027 sim GUI has no timed match. (Revised: the GUI does mirror DS state that
   code sets, so `RobotSim` now acts as FMS, disabling at AUTO and TELEOP end and enabling TELEOP after AUTO; see
   `research-2027-sim-apis.md`.) Originally:
   the rules engine reads enable/mode edges and cuts robot outputs (`SparkSim.disable()`) between AUTO and
   TELEOP and after the match.
7. **Robot code changes stay minimal:** a sim mode switch, hardware accessors, one pose-reset hook, a
   FAST-only auto override, and the `frc.robot.sim` package.
8. **Three sim modes, chosen with `-PsimMode`:**
   - `sim` (default): interactive, real time, GUI.
   - `fast`: headless lockstep for CI. `setUseTiming(false)` plus `SimHooks.pauseTiming()` and
     `stepTiming(0.02)` every loop, so HAL time, PathPlanner and physics stay in sync while running as fast as
     the CPU allows.
   - `replay`: AdvantageKit log replay.

   Untimed without lockstep would leave HAL time on the wall clock and desync everything from the physics.

### Units and frames

SI everywhere. Field frame is WPILib blue-origin (z up), which AdvantageScope's 2026 field also uses.

## Steps

Do them in order. Each step doc lists files, APIs, algorithms, constants, tests and an acceptance check, and ends
with one commit.

| # | Step | Repo | Output |
|---|---|---|---|
| 01 | [Project scaffold](steps/01-project-scaffold.md) | library | Gradle 9.6.1 / Java 25 library, publishes to Maven Local |
| 02 | [Box3D native build](steps/02-box3d-native-build.md) | library | `box3d.dll` from pinned `f36d061`, checked in |
| 03 | [FFM bindings](steps/03-ffm-bindings.md) | library | jextract bindings, native loader, smoke test |
| 04 | [SimWorld API](steps/04-simworld-api.md) | library | math types, backend-neutral world API, Box3D backend, tests |
| 05 | [Motor and battery](steps/05-motor-battery.md) | library | `DcMotor`, `Battery`, `LumpedMechanism` |
| 06 | [Swerve drive](steps/06-swerve-drive.md) | library | `SwerveDriveSim` with tire slip and load transfer, `GyroSim` |
| 07 | [Arena and game pieces](steps/07-arena-gamepieces.md) | library | `Arena` loop, 3D game pieces with drag and Magnus |
| 08 | [REBUILT field](steps/08-rebuilt-field.md) | library | static field geometry, FUEL type, exact starting layout |
| 09 | [REBUILT rules](steps/09-rebuilt-rules.md) | library | match clock, HUB shifts and scoring, HUB exits, OUTPOST, G407 |
| 10 | [Intake, storage, shooter](steps/10-intake-storage-shooter.md) | library | motor-driven mechanisms that move FUEL |
| 11 | [Robot integration](steps/11-robot-integration.md) | robot | `frc.robot.sim`, Spark/Pigeon/battery wiring, logging, sim controls |
| 12 | [Play and calibrate](steps/12-play-and-calibrate.md) | both | AdvantageScope setup, solo play guide, shot calibration, limitations |
| 13 | [Headless match runner](steps/13-headless-match-runner.md) | robot | scripted DS, pass/fail checks, results JSON, `scripts/sim-ci.ps1` |
| 14 | [Suspended chassis](steps/14-suspended-chassis.md) | both | six-DOF chassis over BUMPs, Pigeon pitch/roll, blocked shooter exit, log re-enactment |

Dependencies are strictly linear except that 05 needs only 01, so 05 can be done in parallel with 02–04.

Build the robot repo with `JAVA_HOME=C:\Users\Public\wpilib\2027_alpha7\jdk`; the `java` on PATH is JDK 23.
Verified 2026-10-26: the alpha-7 install builds the unmodified robot project, and `gradlew run` reaches
`Robot program startup complete`.

## How to execute a step (for the implementing model)

1. Read this file, then the step doc in full, then any doc it links. Read the actual source of anything the step
   builds on. Earlier steps' code is the truth when it differs from a later doc's sketch.
2. Implement exactly the files and APIs listed. Where a step says "check the generated class" or "verify",
   look at the real code (jextract output, vendor sources) instead of guessing.
3. Keep **ASSUMPTION** constants as named constants with a short comment naming them as assumptions.
4. Run the step's acceptance commands. All tests must pass before committing. If a test can't pass as written,
   because the physics disagrees with the expected tolerance, stop and report the measured numbers rather than
   loosening the test silently.
5. One commit per step with the message the step gives. Commit only; never push. Library commits go in
   `F:\Projects\box3d-sim` (branch `master`); robot commits go on branch `box3d-sim` in the robot repo.
   Before each commit, check the staged files for secrets and for stray build output.

## Open questions for Kurtis

None of these block implementation; each has a default in the step docs. The answers improve fidelity.

| # | Question | Default used | Where |
|---|---|---|---|
| 1 | Which side is the intake on, and how wide is it? | **answered: back**; 0.53 m roller span (CAD); intake direction only; deploy state ignored | step 11 `SimConstants` |
| 2 | Hopper capacity? | 40 FUEL | step 11 |
| 3 | Flywheel diameter and gearing; hood angle at the hub preset (0) and at −8.5 rotations | **answered (CAD)**: 4 in, Vortex 1:1, 2 in top roller; hood zip-tied at 0 as raced, 78° fixed | steps 11, 12 |
| 4 | Robot height: did it fit under the TRENCH (22.25 in)? | **answered**: yes, filed down to fit; 0.55 m (CAD top 0.583 m predates the filing) | step 11 |
| 5 | Hood, feeder, indexer, intake motors: NEO or NEO 550? Intake pivot gearing? | **motors answered (CAD)**: all NEO. Pivot: NEO, 20:1 planetary, 16T #25 sprocket into an unknown sprocket; 65:1 kept | step 11 |
| 6 | Real drive gearing: the code uses 3.625 for encoders, 3.0 for feedforward, and 4.23 in PathPlanner | 3.625 (matches odometry) | step 11 |
| 7 | Real module spacing: 27.5 in in code vs 0.546 m in PathPlanner | 27.5 in (what kinematics uses) | step 11 |
| 8 | Where did hub shots land from the auto start? Any FUEL bounce or roll observations? | **answered**: the hub preset sank reliably from the auto end point; bounce and roll still unknown | step 12 calibration |
| 9 | `Shoot at Hub` calls `Intake Toggle Run`, but the registered name is `Toggle Intake Run`. Fix it in the robot code? | leave as is | robot repo |
| 10 | Frame ground clearance, and did the raced robot cross BUMPs or always use the TRENCH? | 1 in; it high-centers on the ridge below about 3 V of drive | step 14 `SimConstants` |
| 11 | Real match logs from SoCal Showdown (`.wpilog` from the SystemCore) | **answered**: q10 re-enacts; not used for tuning, since the code the robot ran can't be verified | step 14 re-enact |

## Beyond v0 (not planned in detail)

- Re-enactment that re-syncs the simulated robot to the logged pose and velocity every few seconds, so its error
  measures dynamics over short windows instead of open-loop drift over a whole match.
- A physical intake roller collider and a storage path with jams.
- HUB hex geometry, internal funnel and physical exit chutes.
- Opponent and partner robots (scripted or a second gamepad), towers and climbing.
- Drivetrain compliance and backlash, and shooter wheel compression.
- Linux and macOS natives plus CI.
- A custom AdvantageScope robot model fed by `FieldSimulation/Components`.
