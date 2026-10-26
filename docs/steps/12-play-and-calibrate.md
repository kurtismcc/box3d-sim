# Step 12: AdvantageScope setup, solo play, calibration and wrap-up

**Goal:** a written, repeatable way to launch the sim, watch it in AdvantageScope and play a full REBUILT match solo.
Also: a calibration pass on the shooter assumptions, and an honest list of what v0 does not model.

**Depends on:** step 11.

## 1. Shot calibration harness (library)

Add `src/test/java/box3dsim/season/rebuilt2026/ShotCalibrationTest.java`. It is a test, but a report more than an
assertion.

- **Setup:** an Arena with the field and match, plus a stationary robot (no driving) built from the step 11
  constants. The test copies those constants into a local `ShooterConfig`, so the library has no robot-repo
  dependency.
- **Sweep:** for each preset shot, (2700 rpm, hood 0) and (3000 rpm, hood −8.5 motor rotations):
  - Set the flywheel ω directly with `setState`.
  - Set the hood position directly.
  - Place the robot facing the blue HUB so the **exit point** is at horizontal distance d = 1.0, 1.5, 2.0, 2.5,
    3.0 m from the HUB center. With the exit 0.10 m ahead of the robot center, d = 1.0 leaves the bumper about
    5 cm clear of the HUB face.
  - Launch one FUEL with drag and Magnus on.
- **Record** the FUEL's x-y position when it descends through z = 1.829 (the HUB opening height), or where it lands
  if it never gets that high. Print a table:

  ```
  preset  d   cross_x  cross_y  in_opening  apex_z
  ```

  `in_opening` is |x − 4.6255| < 0.53 and |y − 4.0346| < 0.53.
- **Single assertion:** the hub preset at d = 1.78 m (the `Shoot at Hub` auto's end point, 4.6255 − 2.75 − 0.10)
  has `in_opening == true`.
- **If it fails:** adjust `exitSpeedRatio` (0.35–0.60) first, then the hood pitch mapping (`72° + 2°/rot`) within
  ±10°. Record the chosen values in step 11's `SimConstants` and in this doc's table below. **Ask Kurtis** where the
  real shots landed from the auto start (or from tape marks at SoCal Showdown) before tuning further. The real
  robot is the ground truth, and these constants are guesses.

**Measured with the v0 defaults (2026-10-26):**

| preset | d (m) | crossing x | in opening | apex z |
|---|---|---|---|---|
| hub (2700 rpm, hood 0) | 1.0 | 3.33 (bounced back) | no | 2.62 |
| hub | 1.5 | 4.61 | yes | 2.74 |
| hub | 2.0 | 4.69 | yes | 2.74 |
| hub | 2.5 | 4.19 | yes | 2.74 |
| hub | 3.0 | 3.81 (short) | no | 2.74 |
| side (3000 rpm, −8.5 rot) | 1.0 | 2.91 (bounced back) | no | 2.11 |
| side | 1.5 | 3.44 (bounced back) | no | 1.91 |
| side | 2.0 | 4.76 | yes | 2.59 |
| side | 2.5 | 4.85 | yes | 2.59 |
| side | 3.0 | 4.94 | yes | 2.59 |

The auto-end-point assertion passes. The headless `Shoot at Hub` run (step 13) scores all 8 preloads.

| Constant | v0 default | Calibrated | Notes |
|---|---|---|---|
| flywheel radius | 0.0508 m | | |
| flywheel J | 0.004 kg·m² | | |
| exitSpeedRatio | 0.48 | | |
| hood pitch at 0 rot | 72° | | |
| hood °/motor rot | 2.0 | | |
| FUEL restitution | 0.35 | | drop from 1.83 m onto carpet; real rebound height? |
| carpet rolling resistance | 0.06 | | a FUEL rolled at 2 m/s should stop after ~? m |

## 2. AdvantageScope

**Version:** AdvantageScope v26.0.2 (2026 release) or the v27 alpha that ships with the WPILib 2027 installer. Both
download the same "2026 Field" asset. The machine currently has the 2025 build (`C:\Users\Public\wpilib\2025`),
which predates the 2026 field. Installing WPILib 2027 alpha 7 also provides JDK 25.

Live connection:

1. Start the robot sim (section 3).
2. In AdvantageScope: **File → Connect to Simulator**, choosing NetworkTables (AdvantageKit) if asked. Data appears
   under `/AdvantageKit/RealOutputs/...`.

3D field tab (+ → 3D Field):

1. Field: **2026 Field**. Its coordinate system is "wall-blue", which matches WPILib's blue origin, so no
   conversion is needed.
2. Drag `RealOutputs/FieldSimulation/RobotPose` into Poses and set it as Robot. Model: **2026 KitBot**, which is
   built in. A custom 2026-Offseason model can use `FieldSimulation/Components` later.
3. Drag `RealOutputs/FieldSimulation/Fuel` in and set type **Game Piece**, variant **Fuel**. AdvantageScope hides
   its staged FUEL as soon as these poses arrive, so the sim publishes every FUEL, including chute and corral stock.
4. Optional: drag `RealOutputs/Drive/Pose` in as a **Ghost** to compare odometry with the true pose.

Other tabs:

- Line graph: `FieldSimulation/Robot/FlywheelRPM` vs `Shooter/Actual RPM` (same sign convention; both ≈ +2700 on a
  hub shot), plus `FieldSimulation/Battery/Volts`.
- Table: `FieldSimulation/Match/*` and `FieldSimulation/Outpost/*`.

Save the layout with **File → Export Layout…** to `D:\Projects\2026-Offseason\advantagescope\box3d-sim-layout.json`
and commit it in the robot repo. It's tied to the AdvantageScope major version, so re-export after upgrading.

**Tuning mode:** click the slider icon next to the search bar. Toggling `/Tuning/Sim/*` booleans triggers field
reset and human-player actions without a keyboard joystick.

## 3. Playing a match solo

Add this section, kept in sync, to the robot repo as `SIMULATION.md`:

1. Publish the library: in `F:\Projects\box3d-sim`, run `.\gradlew.bat publishToMavenLocal` after any library
   change.
2. Run the robot: in `D:\Projects\2026-Offseason`, run `.\gradlew.bat run`, or use VS Code → WPILib: Simulate Robot
   Code (Sim GUI). In the sim GUI:
   - Drag the gamepad from **System Joysticks** to **Joysticks** slot 0.
   - Drag **Keyboard 0** to slot 1 for sim controls.
   - Set the alliance station to **Blue 1**. The GUI defaults to Red 1, and PathPlanner flips paths for red.
3. Connect AdvantageScope (section 2).
4. Auto:
   - Choose the auto in the `Auto Path` tunable (default `None`; pick `Shoot at Hub`).
   - Select **Autonomous** and **Enable**.
   - The sim runs 20 s of AUTO. At 0:00 it cuts robot outputs (`FieldSimulation/Match/OutputsAllowed = false`),
     like FMS would.
5. Teleop:
   - Click **Disable**, select **Teleoperated**, **Enable**.
   - The sim runs TRANSITION (10 s), SHIFT 1–4 (25 s each, alternating active HUBs) and END GAME (30 s).
   - Watch `Match/BlueHubActive`: FUEL scored while inactive is worth 0.
   - At 0:00 outputs are cut (POST_MATCH).
6. Human player (keyboard joystick slot 1, buttons 0-based; keys shown in the GUI's Keyboard 0 settings):

   | Button | Action |
   |---|---|
   | 0 | Reset field |
   | 1 | Release chute |
   | 2 | Refill chute from corral |
   | 3 | Throw at hub |

7. Next match: press **reset field**, then go back to step 4. Enabling teleop without auto starts a teleop-only
   practice match.

Controls on the gamepad are the robot's own (see [robot-2026-offseason.md](../robot-2026-offseason.md#controls-robotcontainerconfigurebindings-preset-1)).

## 4. Known v0 limitations

Put these in the library README under `## v0 limitations`:

- **Flat chassis.** The robot is locked to the floor plane, with no roll, pitch or ride height. BUMPs don't affect
  robots (they drive across as if flat), and FUEL still rolls over BUMPs. A robot taller than the trench clearance
  is blocked by the TRENCH arm.
- **Normal loads** come from analytic load transfer, not contact forces.
- **Intake** has no physical roller collider: FUEL in the deployed zone is taken or ignored.
- **Storage** is counted, not simulated. There are no jams or ball-to-ball contact inside the robot.
- **HUB** has a square rim instead of the hex, no internal funnel geometry, and exits are spawn points with random
  speeds.
- **OUTPOST chute** is not physical; released FUEL spawns at the chute mouth.
- **One robot.** No opponents or partners, no towers or climbing, no vision.
- **Shooter constants** are assumptions until calibrated (section 1).
- **REV sim** adds unseeded velocity noise, so runs are not bit-for-bit repeatable. The sim HAL never reports
  brownout, but `FieldSimulation/Battery/BrownedOut` does.
- **Windows x64 natives only.**

## 5. Performance check (report, don't optimize)

Play one full match with the neutral zone stirred up, then read `FieldSimulation/Perf/StepMs`. Report the mean and
the max of the per-loop sum. The budget is well under 20 ms per loop; record the numbers here:

| Metric | Value |
|---|---|
| mean ms per 20 ms loop | not yet measured interactively. Headless lockstep runs whole loops (robot code + 5 physics substeps) at 2.6–3.2× real time, about 6–8 ms per loop |
| max ms per 20 ms loop | one 30 ms loop at startup (JIT warm-up); none after in a 20 s disabled run |
| awake FUEL at max | not yet measured |

If the sim falls behind real time, report it to Kurtis with the numbers. Options include fewer substeps,
`workerCount > 1`, and skipping aerodynamics for slow balls. Don't apply them unasked.

## 6. Wrap-up

- Library `README.md`: under `## Status`, write "v0: plays 2026 REBUILT with the 2026-Offseason robot". Link
  `docs/plan.md` and the limitations.
- Commit in the library: `Document v0 play, calibration and limitations`.
- Commit in the robot repo (branch `box3d-sim`): `SIMULATION.md` and the AdvantageScope layout.
