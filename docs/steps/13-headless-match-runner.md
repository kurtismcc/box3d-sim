# Step 13: Headless match runner (CI)

**Goal:** `.\gradlew.bat run -PsimMode=fast` plays a scripted match with no GUI and no human, faster than real
time. It then exits with code 0 (pass) or 1 (fail) and writes a results file, so a script or CI job can run the
robot's autos and catch regressions.

**Depends on:** step 11 (the FAST mode switch and lockstep timing) and step 12 (calibrated shooter constants, so
score thresholds mean something).

Work in the robot repo, branch `box3d-sim`.

## How FAST mode works (from step 11)

- `setUseTiming(false)`: the loop doesn't wait.
- `SimHooks.pauseTiming()` in `simulationInit()`, then `SimHooks.stepTiming(0.02)` at the end of every
  `simulationPeriodic()`: HAL time advances exactly 20 ms per loop, so `Timer`, PathPlanner, AdvantageKit
  timestamps, the match clock and the physics all agree.
- There's no sim GUI. The GUI never pushes DS state while timing is paused anyway, so this runner owns the
  driver station through `org.wpilib.simulation.DriverStationSim`.

## Options (environment variables, set from Gradle properties by step 11's `build.gradle` block)

| Gradle property | Env var | Default | Meaning |
|---|---|---|---|
| `-PsimAuto=<name>` | `SIM_AUTO` | none (no auto) | PathPlanner auto to run, e.g. `Shoot at Hub` |
| `-PsimMatch=auto\|full` | `SIM_MATCH` | `auto` | `auto`: AUTO plus 3 s scoring grace, then stop. `full`: the whole match through POST_MATCH (teleop idles; nobody drives) |
| `-PsimExpectMinScore=<n>` | `SIM_EXPECT_MIN_SCORE` | `0` | fail if the blue score at the end is below n |

## Robot code change: auto override

The auto chooser is a `Selectable` published through `Tunables`, and nobody can click it headless. In
`RobotContainer.getAutonomousCommand()`, before using the chooser:

```java
String simAuto = System.getenv("SIM_AUTO");
if (simAuto != null && !simAuto.isBlank() && Operating.Constants.simMode() == Operating.Constants.SimMode.FAST) {
    return new PathPlannerAuto(simAuto.trim());
}
```

This only takes effect in FAST mode, so a stray environment variable can't change a real or interactive run.

## `frc.robot.sim.HeadlessMatchRunner`

Give `RobotSim` package-private getters `drive()` (the `SwerveDriveSim`) and `match()` (the `RebuiltMatch`). The
runner needs them for its checks. The robot-side values come from `rc.getDriveSubsystem()`: `getOdometry()` and
`getRotation2d()`.

`Robot.simulationInit()` creates it in FAST mode, after `RobotSim`:
`runner = new HeadlessMatchRunner(robotSim, m_robotContainer);`. `simulationPeriodic()` calls
`runner.periodic()` after `robotSim.periodic()` and before `SimHooks.stepTiming(0.02)`.

### Script (sim time t from the runner's first `periodic()`)

| t (s) | DS action |
|---|---|
| 0 | `setDsAttached(true)`, `setAllianceStationId(BLUE1)`, `setRobotMode(AUTONOMOUS)`, `setEnabled(false)` |
| 0.5 | `setEnabled(true)`. `RebuiltMatch` starts AUTO on this edge, and `Robot.autonomousInit()` schedules the auto |
| 0.5–20.5 | `setMatchTime(20 − (t − 0.5))` |
| 20.5 | `setEnabled(false)`; `RebuiltMatch` has already cut outputs at 20.0 s of AUTO |
| 23.5 | `SIM_MATCH=auto`: finish (the 3 s AUTO scoring grace is over). `full`: `setRobotMode(TELEOPERATED)`, `setEnabled(true)` |
| 23.5–163.5 | `full` only: `setMatchTime(140 − (t − 23.5))` |
| 163.5 + 3 | `full` only: finish (POST_MATCH plus 3 s scoring grace) |

- Call `DriverStationSim.notifyNewData()` after every change, and once every loop anyway. The robot only sees DS
  changes after new data is signalled.
- The enum names come from the alpha-7 imports in `DriverStationSim.java`: `org.wpilib.hardware.hal.RobotMode`
  and `org.wpilib.hardware.hal.AllianceStationID`. Check the constant spellings in the generated sources
  (for example `AUTONOMOUS` / `TELEOPERATED`, `BLUE1` or `kBlue1`) and use what exists.
- Joysticks: none. In `full` mode the robot sits still through teleop. That still tests teleop-start behavior,
  the HUB shift schedule and that nothing throws. Scripted driving is out of scope.

### Checks recorded during the run

| Check | How | Fails the run? |
|---|---|---|
| Score | `match.score(BLUE)` at finish ≥ `SIM_EXPECT_MIN_SCORE` | yes |
| Phase reached | `auto`: phase is AUTO_TO_TELEOP at finish; `full`: POST_MATCH | yes |
| Odometry drift | at the end of AUTO (t = 20.5), distance between `driveSub.getOdometry()` and `drive.pose()` ≤ 0.30 m | yes. This catches gearing, offset and turn-wrapping regressions |
| Heading lag (Phoenix in lockstep) | each loop, `|wrap(gyro yaw read by robot − sim gyro yaw)|`; record the max | no, warn above 2°. See below |
| Sim speed | wall-clock duration and sim duration; record the real-time factor | no |
| Watchdog | sim time > 200 s, or wall time > 10 min (a daemon thread started in the constructor) | yes, exit 2 |

### Finish

1. Write `build/sim-results.json`:

   ```json
   {
     "result": "PASS",
     "mode": "auto",
     "auto": "Shoot at Hub",
     "blueScore": 7, "redScore": 0, "expectMinScore": 3,
     "phase": "AUTO_TO_TELEOP",
     "odometryErrorM": 0.08,
     "maxHeadingLagDeg": 0.6,
     "simSeconds": 23.5, "wallSeconds": 6.1, "realTimeFactor": 3.9,
     "fouls": {"blue": 0, "red": 0},
     "failures": []
   }
   ```

   Use a hand-built string or `org.wpilib`'s bundled Jackson if it's on the classpath; don't add a dependency.
2. Print one line: `SIM RESULT: PASS score=7 odomErr=0.08m rtf=3.9x`, or `FAIL <reasons>`.
3. Flush AdvantageKit: `Logger.end()`, if it exists in 27.0.0-alpha-6; check the source.
4. Exit with `System.exit(0)` on PASS, `1` on FAIL, `2` on watchdog.

## The Phoenix timing caveat

CTRE's sim status signals appear to update on real CAN-frame intervals in wall-clock time (unverified;
`research-2027-sim-apis.md`). In lockstep the sim runs several times faster than real time, so the Pigeon's yaw
may update fewer times per simulated second than it would on the robot. `pigeon.getYaw()` refreshes by default
(`getYaw()` calls `getYaw(true)` in `CorePigeon2`), so the robot gets the latest frame, but that frame can be stale.

The heading-lag check measures this.

- If the max lag stays under 2°, do nothing.
- If it's larger, raise the yaw frame rate in sim: in `RobotSim`'s constructor, FAST mode only, call
  `rc.getDriveSubsystem().getGyro().getYaw().setUpdateFrequency(1000)`. Re-measure.
- If that's still too much, report the numbers to Kurtis. The remaining option is a sim-only heading override in
  `DriveSubsystem`, which is a robot-code design change that needs his decision.

## Local CI script: `scripts/sim-ci.ps1` (robot repo)

1. Fail if `JAVA_HOME` isn't JDK 25. If unset, default to `C:\Users\Public\wpilib\2027_alpha7\jdk`.
2. `& F:\Projects\box3d-sim\gradlew.bat -p F:\Projects\box3d-sim publishToMavenLocal`. Take the library path from
   a `-LibraryPath` parameter, defaulting to that path.
3. For each case, run it and check the exit code:

   | Case | Command |
   |---|---|
   | Auto smoke | `.\gradlew.bat run -PsimMode=fast -PsimAuto="Shoot at Hub" -PsimMatch=auto -PsimExpectMinScore=<calibrated>`. Take the threshold from step 12's calibration; use 1 if it isn't calibrated yet |
   | Full match | `.\gradlew.bat run -PsimMode=fast -PsimMatch=full` (no auto; checks that a full match runs end to end) |

4. Copy each run's `build/sim-results.json` to `build/sim-ci/<case>.json`.
5. Print a summary table. Exit non-zero if any case failed.

A GitHub Actions workflow would need box3d-sim published somewhere the runner can reach (not Maven Local) and a
Windows runner, so it's out of scope for v0. The script is what a workflow would call later.

## Acceptance

1. `.\gradlew.bat run -PsimMode=fast -PsimAuto="Shoot at Hub"` exits on its own with code 0. It prints
   `SIM RESULT: PASS`, the real-time factor is > 1, and `build/sim-results.json` exists.
2. **Lockstep check:** in that run's AdvantageKit log, consecutive `FieldSimulation/Perf/StepMs` entries (logged
   every loop) are 0.02 s apart (± 1e-6). Don't use `Match/TimeRemaining`, which the robot logs only every 0.2 s.
   Also check that `FieldSimulation/Match/Phase` turns AUTO_TO_TELEOP 20.0 s (± 0.02) after the enable.
3. **Failure path:** the same command with `-PsimExpectMinScore=999` exits 1 with `SIM RESULT: FAIL` naming the score.
4. `-PsimMatch=full` reaches POST_MATCH and exits 0. Report its wall time.
5. Interactive mode is unchanged: plain `.\gradlew.bat run` opens the GUI and runs in real time (no
   `HeadlessMatchRunner` is created).
6. `pwsh scripts/sim-ci.ps1` runs both cases and exits 0.

Report the real-time factor and max heading lag from these runs.

Commit (robot repo): `Add headless match runner and sim CI script`.
