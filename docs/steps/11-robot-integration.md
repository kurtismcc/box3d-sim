# Step 11: Plug box3d-sim into the 2026-Offseason robot

**Goal:** the robot at `D:\Projects\2026-Offseason` runs in desktop simulation against box3d-sim, with every motor,
the gyro and the battery driven by the physics, and it publishes the field state for AdvantageScope.

**Depends on:** steps 01–10, published with `.\gradlew.bat publishToMavenLocal` from `F:\Projects\box3d-sim`.

## Ground rules

- Work on a new branch in the robot repo: `git -C D:\Projects\2026-Offseason switch -c box3d-sim alpha7`. Commit
  there. Never push.
- Robot logic (subsystems, commands, bindings, tuning) stays as it was at SoCal Showdown. Changes are limited to:
  1. the sim mode switch (interactive, fast, replay);
  2. accessors so sim code can reach the hardware objects;
  3. one pose-reset hook;
  4. the new `frc.robot.sim` package.
- Only `frc.robot.sim` imports `box3dsim.*`. Nothing else in the robot imports the library.

## Facts this step relies on (WPILib 2027.0.0-alpha-7 and vendors, verified from sources)

| Topic | Fact |
|---|---|
| Running sim | GradleRIO alpha-7 runs Java sim with the application `run` task (`.\gradlew.bat run`) or VS Code "WPILib: Simulate Robot Code". `includeDesktopSupport` is an unused variable, so leave it. GradleRIO already passes `--enable-native-access=ALL-UNNAMED` |
| Prerequisite | JDK 25 on `JAVA_HOME`. The WPILib 2027 alpha 7 installer provides one under `C:\Users\Public\wpilib\2027_alpha7\jdk` |
| Battery | `org.wpilib.simulation.RoboRioSim.setVInVoltage(double)`. `RobotController.getBatteryVoltage()` reads it. The sim HAL never reports brownout |
| DCMotor | `org.wpilib.math.system.DCMotor`: `getNeoVortex(n)`, `getNEO(n)`, `getNeo550(n)`, `withReduction(r)` |
| REV sim | `com.revrobotics.sim.SparkMaxSim(SparkMax, DCMotor)` and `com.revrobotics.sim.SparkFlexSim(SparkFlex, DCMotor)`, both extending `com.revrobotics.spark.SparkSim`, which has the methods below |
| `SparkSim` methods | `iterate(double velocityRPM, double vbus, double dt)`, `getAppliedOutput()`, `getSetpoint()`, `setPosition`, `getPosition`, `setVelocity`, `getMotorCurrent`, `enable()`, `disable()`, `useDriverStationEnable()`, `getRelativeEncoderSim()`, `getAbsoluteEncoderSim()` |
| `iterate` | Runs REV's native port of the firmware closed loop (PID, kV, output range, MAXMotion, smart current limit). Integrates position from the velocity you pass, then mirrors position and velocity into the **closed-loop feedback sensor only**. Zeroes output when the robot is disabled. Ignores idle mode and inversion |
| Encoder sims | `SparkRelativeEncoderSim` and `SparkAbsoluteEncoderSim` have `setPosition`, `getPosition`, `setVelocity`, `iterate(rpm, dt)`. The absolute sim does **not** wrap |
| Pigeon 2 | `pigeon.getSimState()` returns `Pigeon2SimState`: `setRawYaw(deg)` (continuous), `setAngularVelocityZ(dps)`, `setSupplyVoltage(V)` (≥ 4) |
| Sim time | `org.wpilib.simulation.SimHooks`: `pauseTiming()`, `resumeTiming()`, `stepTiming(double seconds)`. `stepTiming` waits for signalled notifiers, then advances HAL time. AdvantageKit's `LoggedRobot` arms no notifier when timing is off, so calling it from the robot thread is safe (inferred from source; step 13 verifies it) |
| DS sim | `org.wpilib.simulation.DriverStationSim`. The sim GUI mirrors DS state that code sets: it reads HAL each frame and writes only on clicks. So `RobotSim` acts as FMS in both modes (disable at AUTO and TELEOP end, enable TELEOP 3 s after AUTO). Match time is the exception: the GUI rewrites it while enabled. Corrected 2026-10-26; this row used to say the GUI overwrote code-set state. Read the state with `RobotState.isEnabled()`, `isAutonomous()`, `isTeleop()` (or `org.wpilib.driverstation.DriverStation` equivalents; use whatever `Robot.isTeleopEnabled()` uses in `DriveSubsystem`) |
| Alliance | The sim GUI defaults to **RED 1**. `MatchState.getAlliance()` returns `Optional<Alliance>` |
| Joysticks | The sim GUI maps a USB gamepad to port 0 by dragging it from "System Joysticks" to slot 0. HAL button indices are 0-based. The GUI does not send joystick data while in Autonomous |
| AdvantageKit | `setUseTiming(false)` makes the loop run back-to-back with microsecond dt. `LogFileUtil.findReplayLog()` silently enters replay if AdvantageScope's temp file exists, and blocks on stdin under VS Code |

## 1. Build changes: `build.gradle`

Add this above `dependencies` (GradleRIO keeps its own repositories):

```groovy
repositories {
    mavenLocal()
}
```

and inside `dependencies`:

```groovy
implementation 'box3dsim:box3d-sim:0.1.0-SNAPSHOT'
```

Leave `includeDesktopSupport` alone.

### Sim mode selection

The sim mode comes from a Gradle property, defaulting to `sim`. Replace the existing
`wpi.sim.addGui().defaultEnabled = true` line with:

```groovy
// Sim mode: sim (interactive, real time, GUI), fast (headless lockstep, CI), replay (AdvantageKit log replay)
def simMode = (project.findProperty('simMode') ?: 'sim').toString()
wpi.sim.addGui().defaultEnabled = (simMode == 'sim')
tasks.named('run') {
    environment 'SIM_MODE', simMode
    // Headless match options (step 13); passed through only when given.
    ['simAuto': 'SIM_AUTO', 'simMatch': 'SIM_MATCH', 'simExpectMinScore': 'SIM_EXPECT_MIN_SCORE']
        .each { prop, env -> if (project.hasProperty(prop)) environment env, project.property(prop).toString() }
}
```

| Command | Mode |
|---|---|
| `.\gradlew.bat run` | interactive |
| `.\gradlew.bat run -PsimMode=fast -PsimAuto="Shoot at Hub"` | headless CI run (step 13) |
| `.\gradlew.bat run -PsimMode=replay` | AdvantageKit replay |

VS Code's "Simulate Robot Code" doesn't set `SIM_MODE`, so it gets interactive, which is the desired default.

## 2. `Robot.java`: three sim modes

Add inside the nested `Operating.Constants` class in `Constants/Operating.java`:

```java
public enum SimMode { SIM, FAST, REPLAY }
/** From the SIM_MODE environment variable (set by build.gradle); SIM when absent or unrecognized. */
public static SimMode simMode() {
    String v = System.getenv("SIM_MODE");
    if (v == null) return SimMode.SIM;
    return switch (v.trim().toLowerCase()) {
        case "fast" -> SimMode.FAST;
        case "replay" -> SimMode.REPLAY;
        default -> SimMode.SIM;
    };
}
```

Then in `Robot()`, replace the non-real branch:

| Mode | Behavior |
|---|---|
| Real | unchanged |
| `SIM` (interactive) | `Logger.addDataReceiver(new WPILOGWriter("logs"))`; NT4 is already added above. Keep timing on: **do not** call `setUseTiming(false)`. **Do not** call `findReplayLog()` |
| `FAST` (headless lockstep) | `setUseTiming(false)` and `Logger.addDataReceiver(new WPILOGWriter("logs"))`. Keep NT4 too: it's harmless headless and lets AdvantageScope watch a fast run live. No `findReplayLog()` |
| `REPLAY` | the existing replay block: `setUseTiming(false)`, `findReplayLog`, `WPILOGReader`, and the `_sim` writer |

**Why FAST needs lockstep.** `setUseTiming(false)` only stops the loop from waiting; HAL time stays the wall
clock. On its own it would make every loop a few microseconds of HAL time while the physics advances 20 ms, so
`Timer`, PathPlanner and the match clock would fall far behind the robot. FAST therefore freezes HAL time and
advances it by exactly one period per loop with `org.wpilib.simulation.SimHooks`. The whole program then sees a
consistent 20 ms loop and runs as fast as the CPU allows.

Add the fields `private frc.robot.sim.RobotSim robotSim;` and `private frc.robot.sim.HeadlessMatchRunner runner;`
(the runner is step 13; leave it null until then). Then:

- `simulationInit()`:
  - `SimMode mode = Operating.Constants.simMode();`
  - If `mode == FAST`: `SimHooks.pauseTiming();`
  - If `mode != REPLAY`: `robotSim = new RobotSim(m_robotContainer);`
  - Step 13 adds the runner construction here, for FAST only.
- `simulationPeriodic()`:
  1. `if (robotSim != null) robotSim.periodic();`
  2. `if (runner != null) runner.periodic();` (step 13)
  3. If FAST, last: `SimHooks.stepTiming(0.02);`

`simulationPeriodic` runs after `robotPeriodic()` (the command scheduler) in every loop. Commands set Spark
setpoints first, physics follows in the same loop, and in FAST mode HAL time then advances one period.

**FAST acceptance for this step** (the full headless match is step 13):

```
.\gradlew.bat run -PsimMode=fast
```

The sim GUI does **not** open, and the console reaches `Robot program startup complete`. After 30 s of wall time,
check the current AdvantageKit log in `logs\`: consecutive `FieldSimulation/Perf/StepMs` entries (logged every
loop) are 0.02 s apart in log time, and the log covers more sim time than the wall time elapsed. Stop
the process. Before step 13 nothing drives the DS, so the robot stays disabled and the run never ends on its own.

## 3. Accessors and the pose hook (small, mechanical edits)

| File | Add |
|---|---|
| `components/PIDMotorIO.java` | `default com.revrobotics.spark.SparkBase getSpark() { return null; }` |
| `components/PIDMotorIOSparkMax.java`, `PIDMotorIOSparkFlex.java` | `@Override public SparkBase getSpark() { return motor; }` |
| `components/PIDMotor.java` | `public PIDMotorIO getIO() { return io; }` |
| `components/SwerveModuleIO.java` | `default PIDMotor getDriveMotor() { return null; }`, `default PIDMotor getTurnMotor() { return null; }` |
| `components/SwerveModuleIONEO.java` | overrides returning `driveMotor` / `turnMotor`, plus `public double getChassisAngularOffset()` |
| `components/SwerveModule.java` | `public SwerveModuleIO getIO() { return io; }` |
| `subsystems/DriveSubsystem.java` | `public SwerveModule[] getModules()` returning `{frontLeft, frontRight, backLeft, backRight}`; `public Pigeon2 getGyro()`; and a pose hook (below) |
| `subsystems/ShooterSubsystem.java` | `getFlywheelMotor()`, `getHoodMotor()`, `getFeederMotor()`, `getIndexerMotor()` returning the `PIDMotor`s |
| `subsystems/GroundIntakeSubsystem.java` | `getIntakeMotor()`, `getDropMotor()` |
| `RobotContainer.java` | `getDriveSubsystem()`, `getShooterSubsystem()`, `getIntakeSubsystem()` |

Pose hook in `DriveSubsystem`:

```java
private java.util.function.Consumer<Pose2d> poseResetListener = pose -> {};
public void setPoseResetListener(java.util.function.Consumer<Pose2d> listener) { poseResetListener = listener; }
```

Call `poseResetListener.accept(pose)` at the end of both `resetOdometry(Pose2d)` and `resetPose(Pose2d)`.
PathPlanner autos with `resetOdom: true` then teleport the simulated robot to the path start, which matches what
happens when the drive team places the robot.

## 4. `frc.robot.sim` package

### `SimConstants.java`: robot presets (ASSUMPTIONS marked)

| Group | Values |
|---|---|
| Swerve | Same numbers as `TestConfigs.maxSwerve()` in step 06: module positions ±0.3493 (from `Drive.Constants`), Vortex drive at gearing **3.625** (`DRIVING_FACTOR`, so encoder conversions are exact), wheel radius 0.0381, wheel MOI 0.002, drive friction 0.15 N·m, NEO 550 steer at gearing 46.42, steer MOI 0.004, steer friction 0.1. Mass 50, yaw MOI 6.883, cg 0.20, bumper 0.9 × 0.9 (bottom 0.0635, height 0.0825), robot height **0.55 (ASSUMPTION; ≤ 0.565 m trench clearance)**, frame inset 0.08, `TireConfig.defaults()`, `GyroConfig.pigeon2()`, load transfer on |
| Flywheel | Vortex 1:1 (ASSUMPTION), J = 0.004 (ASSUMPTION: two 4 in wheels and a shaft), friction 0.05 N·m, viscous 1e-4. Radius 0.0508 (4 in, ASSUMPTION). Shot direction −1 (code: negative RPM shoots). Exit ratio 0.48, spin ratio 0.8, efficiency 0.7, min exit speed 2.0 m/s |
| Hood | Motor NEO 550 (ASSUMPTION). Gearing 180 motor rotations per hood rotation (so 1 motor rotation = 2°, ASSUMPTION). MOI 0.01, friction 0.2 N·m. Limits −12 to +0.5 motor rotations, converted to hood rad. Pitch = 72° + 2°·(motor rotations), so the hub shot (0) is 72° and the side shot (−8.5) is 55° (ASSUMPTION: calibrate in step 12). With this gearing one motor rotation is 2° of hood, so `hoodPositionToPitch = p -> Math.toRadians(72) + p` (p is the hood angle in rad). The exit point (0.10, 0, 0.64) puts the FUEL's bottom (0.565) above the 0.55 m superstructure, with exit yaw 0 (robot front: the `Shoot at Hub` auto faces the hub) |
| Indexer | NEO 550 (ASSUMPTION), gearing 5, J = 5e-4, roller radius 0.025, direction sign −1 (code feeds with −8 V), efficiency 0.8, min surface speed 0.3 |
| Feeder | NEO 550 (ASSUMPTION), gearing 3, J = 3e-4, roller radius 0.025, direction −1 (code feeds with −11 V), efficiency 0.8, min surface speed 0.5 |
| Storage | capacity **40 (ASSUMPTION)**, preload 8 [G303] |
| Intake roller | NEO 550 (ASSUMPTION), gearing 3, J = 5e-4, radius 0.03, direction −1 (`IntakeRun` uses −0.9), efficiency 0.7, min surface speed 0.8 |
| Intake pivot | NEO (ASSUMPTION), gearing 65.14 (−19 motor rotations = 105° of travel, ASSUMPTION), J = 0.15, friction 0.5 N·m. Limits from −19.5 to +0.3 motor rotations, converted to mechanism rad (negative = down). Deployed at −19 rotations, stowed at 0, deployed fraction 0.9 |
| Intake zone | on the robot **back** (confirmed by Kurtis 2026-10-26), across the full bumper width: center (−0.55, 0, 0.075), half extents (0.10, 0.45, 0.10). Deployed fraction 0: FUEL touching the back bumper is taken whenever the roller spins in the intake direction, whatever the pivot state |
| Start pose | blue (3.5, 4.0545, 0) is the `Backup Center` path start (y = 4.054489); red = `mirror` with yaw π |

### `SparkBinding.java`: one Spark ↔ one simulated degree of freedom

```java
final class SparkBinding {
    enum Feedback { RELATIVE, ABSOLUTE }
    SparkBinding(SparkBase spark, DCMotor dcMotorForSim, Feedback feedback, boolean coastIdle);
    /** Step A (before physics): run REV's controller for this substep and return motor volts. */
    double iterate(double feedbackRpm, double vbus, double dt);
    /** Once per robot loop, before any iterate: pick up encoder resets done by robot code. */
    void detectRobotReset();
    /** Step D (after physics): write true sensor positions plus the reset offset. */
    void writeRelative(double motorRotations, double motorRpm);
    void writeAbsolute(double rotations01, double rpm);
    void setOutputsAllowed(boolean allowed);   // false → disable(); true → useDriverStationEnable()
    boolean coast();
}
```

It builds a `SparkMaxSim` or `SparkFlexSim` from the concrete type (`instanceof SparkFlex`). Then:

- **iterate:** `sim.iterate(feedbackRpm, vbus, dt)`, then return `sim.getAppliedOutput() * vbus`.
- **Robot-side encoder resets:** the robot calls `encoder.setPosition(0)` at construction (`resetEncoder`) and
  through `setEncoder`. Writes always go out as `truth + offset`; remember the last value written.
  - Detect resets **once per robot loop**, in a method `detectRobotReset()` that `RobotSim.periodic()` calls for every
    binding **before the first substep's `iterate`**. That's the only window in which robot code ran since the
    last write.
  - In it, read `relEncSim.getPosition()`. If it differs from the last written value by more than 1e-9, the robot
    changed it: `offset += read − lastWritten`.
  - Never compare inside the substep loop. `iterate` itself moves the mirrored position, and comparing there would
    double-count travel.
  - When the feedback is RELATIVE, write `sim.setPosition(truth + offset)` together with the encoder sim.

### Per-motor wiring

`DCMotor` arguments: when the closed-loop feedback is the motor's relative encoder, pass the bare motor. For
absolute-encoder feedback, pass `motor.withReduction(G)`, because `iterate`'s velocity is the feedback sensor's
velocity and REV computes current from it.

| Motor | DCMotor for sim | Feedback velocity passed to `iterate` | After physics |
|---|---|---|---|
| Drive ×4 (SparkFlex) | `getNeoVortex(1)` | wheel ω·3.625·60/2π (motor RPM) | relative: wheel φ·3.625/2π rotations |
| Turn ×4 (SparkMax, absolute feedback) | `getNeo550(1).withReduction(46.42)` | steer ω·60/2π (module RPM). **Before** `iterate`, set `sim.setPosition(pv)` where `a = wrap01((θ_s + offset)/2π)`, `sp = sim.getSetpoint()`, `pv = sp + wrapHalf(a − sp)` and `wrapHalf` maps into (−0.5, 0.5] | absolute: `wrap01((θ_s + offset)/2π)` and module RPM. Relative: θ_s·46.42/2π and motor RPM, so `turnVelocityRadPerSec` logs something |
| Flywheel (SparkFlex) | `getNeoVortex(1)` | flywheel ω·60/2π (motor = flywheel, 1:1) | relative position |
| Hood (SparkMax) | `getNeo550(1)` | hood ω·180·60/2π | relative: hood rad·180/2π |
| Feeder, Indexer (SparkMax) | `getNeo550(1)` | roller ω·G·60/2π | relative |
| Intake roller (SparkMax) | `getNeo550(1)` | roller ω·3·60/2π | relative |
| Intake drop (SparkMax, MAXMotion on relative) | `getNEO(1)` | pivot ω·65.14·60/2π | relative: pivot rad·65.14/2π (start 0 = stowed) |

`offset` is the module's `getChassisAngularOffset()`. The robot computes `turnPositionRad = abs·2π − offset`, so
abs = (θ_s + offset)/2π.

The turning `pv` trick keeps the controller error within half a turn whether or not REV's native sim wraps
kPosition error. Its pv is the unbounded integrated position, and the 2027 API has no wrap input range. The robot
still reads a [0, 1) absolute angle.

**Inversion:** REV sim ignores the inverted flags. Treat "positive applied output" as the robot's commanded
positive direction for every motor:

- Drive: positive = wheel rolls toward the module heading (that's what FL/BL inversion achieves on the real robot).
- Turn: positive = CCW.

The physics models use those directions directly.

**Idle modes** (for `coastAtZero`), from `Configs.java`:

| Coast | Brake |
|---|---|
| flywheel, indexer, intake roller | drive, turn, hood, feeder, drop |

### `RobotSim.java`

Fields: `Arena arena`, `SwerveDriveSim drive`, `IntakeSim intake`, `FuelStorage storage`, `ShooterSim shooter`,
`RebuiltMatch match`, `RebuiltFieldHandles field`, the `SparkBinding`s, `Pigeon2SimState pigeon`, and
`box3dsim.arena.Alliance alliance`. Two `Alliance` types are in play (`org.wpilib.driverstation.Alliance` and
`box3dsim.arena.Alliance`), so import neither; write both fully qualified, or convert in one helper method.

**Constructor (`RobotContainer rc`):**

1. `arena = new Arena(ArenaConfig.defaults())`, giving 4 ms physics with 5 substeps per 20 ms loop.
2. `field = RebuiltField.build(arena.world())` and `match = new RebuiltMatch(arena, field, RebuiltMatch.Config.defaults())`.
3. `drive = new SwerveDriveSim(...)` at the start pose for the current alliance (default BLUE if absent).
4. Create the intake, storage and shooter. Shooter launch listener:
   `(pose, v) -> match.onLaunch(alliance, pose, 0.45, 0.45)`.
5. Add participants **in this order**: drive, intake, storage, shooter, match. Then add loads: drive, intake,
   storage, shooter.
6. Create bindings for all 14 Sparks from the accessors in section 3.
7. Get `pigeon = rc.getDriveSubsystem().getGyro().getSimState()`.
8. `rc.getDriveSubsystem().setPoseResetListener(p -> drive.teleport(p.getX(), p.getY(), p.getRotation().getRadians()))`.
9. `arena.reset()`.
10. Seed the robot's estimators so odometry starts where the sim robot is: `rc.getDriveSubsystem().resetOdometry(startPose)`
    and `resetPose(startPose)`. These also fire the pose listener, which teleports the drive to the same pose.
11. Create the sim controls (below).

**`periodic()`, once per 20 ms loop:**

1. Update `alliance` from `MatchState.getAlliance()` (default BLUE).
2. Feed the DS state: `match.setDsState(new DsState(enabled, auto, teleop))`.
3. `boolean allowed = match.outputsAllowed();` and push `setOutputsAllowed(allowed)` to every binding.
4. `vbus = arena.battery().voltage()`, then call `detectRobotReset()` on every binding.
5. Repeat 5 times (dt = 0.004):
   - (A) For each binding, `volts = binding.iterate(feedbackRpm(from the model's current state), vbus, dt)`.
   - (B) Hand the volts to the models: `drive.setModuleInputs(i, driveVolts, coast, turnVolts, coast)`,
     `intake.setRollerInput/setPivotInput`, `storage.setIndexerInput/setFeederInput`,
     `shooter.setFlywheelInput/setHoodInput`.
   - (C) `arena.step()`.
   - (D) Write sensors back through each binding, from the models' new state.
   - `vbus = arena.battery().voltage()`.
6. `RoboRioSim.setVInVoltage(vbus)`.
7. Pigeon: `pigeon.setSupplyVoltage(vbus)`, `pigeon.setRawYaw(Math.toDegrees(drive.gyro().yawRad()))`, and
   `pigeon.setAngularVelocityZ(Math.toDegrees(drive.gyro().yawRateRadPerSec()))`.
8. Handle the sim controls.
9. Log (below).

### Sim-only controls (solo play)

There is no second driver, so the human player and field reset get two sources. Use whichever fires.

1. **Keyboard joystick on port 1.** Create `CommandGenericHID simOps = new CommandGenericHID(1)`; the robot doesn't
   use port 1. In the sim GUI, drag "Keyboard 0" to slot 1. Buttons are 0-based:

   | Button | Action |
   |---|---|
   | 0 | Reset field (`arena.reset()`), teleport the drive to the alliance start pose, and call `rc.getDriveSubsystem().resetOdometry(startPose)` and `resetPose(startPose)` |
   | 1 | Release chute (own alliance) |
   | 2 | Refill chute from corral |
   | 3 | Human player throws at the hub |

   Bind them with `simOps.button(n).onTrue(new InstantCommand(...).ignoringDisable(true))`. Without
   `ignoringDisable(true)` they won't run while disabled, which is exactly when you reset between matches. Check
   the method name in `org.wpilib.command2.Command`. The GUI's "Keyboard 0 Settings" window shows which keys map
   to these buttons.
2. **AdvantageScope tuning toggles.** `org.littletonrobotics.junction.networktables.LoggedNetworkBoolean`
   publishes `/Tuning/Sim/ResetField`, `/Tuning/Sim/ReleaseChute`, `/Tuning/Sim/RefillChute` and
   `/Tuning/Sim/ThrowAtHub`. Each loop, if a value is true, run the action and `set(false)`.

### Logging (AdvantageKit `Logger.recordOutput`)

| Key | Value |
|---|---|
| `FieldSimulation/RobotPose` | `Pose3d` true pose, converted from `Pose3` |
| `FieldSimulation/RobotPose2d` | `Pose2d` true pose |
| `FieldSimulation/Fuel` | `Pose3d[]` from `match.fuelDisplayPoses()` (**all** FUEL, so AdvantageScope's staged FUEL hides correctly) |
| `FieldSimulation/Components` | `Pose3d[]` robot-relative: [0] intake pivot, a rotation about robot +y by the pivot angle at (0.35, 0, 0.20); [1] hood, a rotation about +y by (pitch − 72°) at (0.10, 0, 0.55). These are placeholders for a future custom robot model |
| `FieldSimulation/Match/Phase`, `/MatchTime`, `/PhaseTime`, `/OutputsAllowed` | strings and doubles |
| `FieldSimulation/Match/BlueScore`, `/RedScore`, `/BlueHubActive`, `/RedHubActive`, `/FirstInactive` | |
| `FieldSimulation/Match/BlueFouls`, `/RedFouls`, `/LastFoul` | |
| `FieldSimulation/Outpost/BlueChute`, `/BlueCorral`, `/RedChute`, `/RedCorral` | |
| `FieldSimulation/Robot/Hopper`, `/Staged`, `/ShotsFired`, `/IntakeAcquired`, `/IntakeDeployProgress` | |
| `FieldSimulation/Robot/FlywheelRPM` | `−flywheel.velocity()·60/2π`. Negated to match the robot's own `Shooter/Actual RPM`, which logs `-flywheelMotor.getVelocity()`, so both read about +2700 on a hub shot |
| `FieldSimulation/Robot/LaunchPitchDeg` | |
| `FieldSimulation/Battery/Volts`, `/Amps`, `/BrownedOut` | |
| `FieldSimulation/Perf/StepMs` | the sum of `arena.lastStepStats()` across the 5 substeps |

Write a small `PoseConversions` with `toPose3d(Pose3)`, `toPose2d(Pose3)` and `fromPose2d(Pose2d)` using
`org.wpilib.math.geometry.*`: `Rotation3d` from the quaternion with `new Rotation3d(new Quaternion(w, x, y, z))`.

## Acceptance

From `D:\Projects\2026-Offseason`, with `JAVA_HOME=C:\Users\Public\wpilib\2027_alpha7\jdk`:

```
.\gradlew.bat build
```

It compiles with the library from Maven Local. Then:

```
.\gradlew.bat run
```

The sim GUI opens with no exceptions in the console for 30 s while disabled. Next, with the gamepad on port 0,
Blue 1 selected, Teleoperated and Enable:

1. The left stick drives the robot. `Drive/Pose` (odometry) tracks `FieldSimulation/RobotPose2d` within 0.2 m after
   20 s of driving. A larger gap points to a gearing or offset mismatch; recheck the 3.625 gearing and the turn
   offsets.
2. Wheels point where commanded: `Drive/ModuleStates/Actual` follows `Desired` within 5° **modulo 180°** after
   0.2 s. `Desired` is logged before `optimize`, so a flipped module is correct.
3. A deploys the intake (`IntakeDeployProgress` → 1). `extend()` already sets the roller to +0.2 and marks it
   running, so the first X press stops it and the second starts it at −0.9 (intake direction). Driving into
   neutral-zone FUEL raises `Robot/Hopper`.
4. B spins the flywheel to about 2700 RPM (`Shooter/Actual RPM` ≈ +2700, same as `FieldSimulation/Robot/FlywheelRPM`),
   and FUEL launches as soon as it's ready (`ShotsFired` climbs). FUEL bodies fly in AdvantageScope.
5. Disable and press **reset field** (keyboard button 0 or `/Tuning/Sim/ResetField`), because the teleop run above
   ended the match in POST_MATCH with outputs cut. Then run Autonomous with `Shoot at Hub` selected. The robot
   teleports to (3.5, 4.05), backs up and shoots, and the blue score rises (at least 1 point; otherwise check the
   shooter assumptions in step 12).

Commit (robot repo, branch `box3d-sim`): `Run 2026-Offseason robot against box3d-sim`.
