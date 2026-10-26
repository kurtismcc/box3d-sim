# WPILib 2027 alpha 7 and vendor simulation APIs

Researched 2026-10-26 from sources at these exact versions:

- allwpilib, GradleRIO and vscode-wpilib `v2027.0.0-alpha-7`
- REVLib-java `2027.0.0-alpha-8` sources and driver headers
- Phoenix 6 `wpiapi-java-26.70.0-alpha-2` sources
- AdvantageKit `27.0.0-alpha-6`
- PathPlannerLib `2027.0.0-alpha-4.1`
- LimelightLib `2.0.0-beta9-alpha7`

These are the versions `D:\Projects\2026-Offseason` uses. "Inferred" marks conclusions not read directly from source.

## GradleRIO / running sim

- `includeDesktopSupport` in `build.gradle` is an unused variable for Java. Only VS Code's "Change Desktop Support"
  command reads it. Desktop JNI comes from the `nativeDebug` / `nativeRelease` lines regardless.
- There is no `simulateJava` task. Java sim is the application **`run`** task: `wpi.java.configureApplication`
  extracts natives to `build/jni/release` and sets `--enable-native-access=ALL-UNNAMED`, `--add-opens` and
  `HALSIM_EXTENSIONS`. VS Code uses `simulateExternalJava`.
- `wpi.sim.addGui().defaultEnabled = true` loads `halsim_gui`. `addDriverstation()` (`halsim_ds_socket`) is
  **not** loaded unless `.defaultEnabled = true`.
- WPILib alpha-7 Maven artifacts live in `https://frcmaven.wpi.edu/artifactory/release/`.

## Sim GUI behavior

- Joysticks: drag a device from "System Joysticks" to a "Joysticks" slot. Assignments are saved by SDL GUID.
  - Gamepads use the SDL standard mapping. Axes: LEFT_X=0, LEFT_Y=1, RIGHT_X=2, RIGHT_Y=3, LT=4, RT=5.
  - HAL buttons are **0-based**; the `getRawButton` "starting at 1" javadoc is stale.
  - Keyboard 0/1/2 virtual joysticks exist.
  - **Joysticks are not sent in Autonomous.**
- "Robot State" starts Disconnected. Picking a mode attaches the DS.
- **No timed match:** match time is −1 unless typed into the FMS window. Even then it only counts down; there is
  no auto→teleop transition and no disable.
- **Default alliance is RED 1.**
- **Corrected 2026-10-26** (the earlier note here was wrong): every frame the GUI **reads** the DS state from HAL
  (`DSSimModel::Update()`). It writes HAL (`UpdateHAL()`) only when you click a control.
  - So `DriverStationSim.setEnabled` and `setRobotMode` from code stick, and both the GUI and AdvantageKit show
    them. box3d-sim's `RobotSim` uses this to act as FMS.
  - The exception is match time. While enabled, the GUI recomputes it from its own clock and writes it every frame
    (−1 unless a time was typed in), so code can't set it.

## WPILib sim classes

| Need | API |
|---|---|
| Battery voltage | `org.wpilib.simulation.RoboRioSim.setVInVoltage(double)`. `RobotController.getBatteryVoltage()` reads it. The default is 12.0 |
| Brownout | the sim HAL never reports brownout (`HAL_GetBrownedOut` returns false) |
| Battery helper | `org.wpilib.simulation.BatterySim.calculateLoadedBatteryVoltage(nominal, resistance, currents...)` |
| Motor constants | `org.wpilib.math.system.DCMotor`, with fields `nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed, R, Kv, Kt`, factories `getNEO`, `getNeo550`, `getNeoVortex`, and `withReduction` |
| Time | `org.wpilib.simulation.SimHooks.pauseTiming/resumeTiming/stepTiming(seconds)` |
| DS | `org.wpilib.simulation.DriverStationSim` (`setEnabled`, `setRobotMode`, `setAllianceStationId`, `setMatchTime`, `notifyNewData`) |
| Loop order | `IterativeRobotBase.loopFunc`: DS refresh → mode periodic → `robotPeriodic` → tunables → `simPeriodicBefore` → `simulationPeriodic` → `simPeriodicAfter` → NT flush |

## REVLib 2027.0.0-alpha-8

- `com.revrobotics.spark.SparkSim(SparkBase, DCMotor)`. Subclasses: `com.revrobotics.sim.SparkMaxSim` and
  `SparkFlexSim`.
- `iterate(velocityRPM, vbus, dt)`:
  1. Adds noise to the velocity (unseeded) and filters it.
  2. Integrates position.
  3. Computes output by control mode. Velocity, position and current modes, plus MAXMotion, run in native code
     ported from the firmware, according to the javadoc.
  4. Adds arbFF and applies voltage compensation.
  5. Applies soft limits and limit switches, then the smart current limit (native).
  6. Sets motor current from `DCMotor`.
  7. Zeroes output when the robot is disabled.
  8. Mirrors position and velocity into the **closed-loop feedback sensor only**.
- Not simulated: idle mode, secondary current limit, inversion. Ramp rate is unknown.
- **Position wrapping:** only `positionWrappingEnabled`. Conversion factors and `positionWrappingInputRange` are
  **removed** in 2027, and the "hardware manager" comments in the robot's `Configs.java` don't match anything in
  REVLib. Setpoint units are RPM, rotations, volts and amps. The absolute encoder default range is [0, 1).
- **Quirk:** voltage compensation is also applied in kVoltage mode, which double-divides by vbus.
- **Signal API:** `getPosition()`, `getVelocity()` and similar return `com.revrobotics.util.Signal<Double>` with
  `get()`. `setThrottle` replaces `set`.

## Phoenix 6 26.70.0-alpha-2

- `Pigeon2.getSimState()` returns `Pigeon2SimState`, with `setSupplyVoltage` (≥ 4 V), `setRawYaw(deg)`
  (continuous), `addYaw`, `setPitch`, `setRoll` and `setAngularVelocityX/Y/Z(dps)`.
- Sim state is keyed by device ID only.
- `new CANBus(CANPort.CAN_S0)` maps to `"can_s0"`, which is fine in sim.
- `AutoFeedEnable` starts automatically in sim.

## AdvantageKit 27.0.0-alpha-6

- `setUseTiming(false)` runs loops back-to-back and does **not** advance sim time. It is for replay only.
- `LogFileUtil.findReplayLog()` checks three things in order:
  1. env `AKIT_LOG_PATH`;
  2. `<tmp>/akit-log-path.txt`, which AdvantageScope writes, so sim may silently replay;
  3. a blocking stdin prompt.
- `Logger.recordOutput(String, T...)` takes `T extends StructSerializable`. `Pose3d` and `SwerveModuleVelocity`
  qualify.
- `@AutoLog` comes from `org.littletonrobotics.akit:akit-autolog:27.0.0-alpha-6`.
- NT4 data lands under `/AdvantageKit/RealOutputs/...`.
- Only `/Tuning/...` keys are writable from AdvantageScope tuning mode. Publish them with
  `LoggedNetworkBoolean` / `LoggedNetworkNumber`.

## PathPlannerLib 2027.0.0-alpha-4.1

- Nothing sim-specific. `RobotConfig.fromGUISettings()` reads `src/main/deploy/pathplanner/settings.json` in sim.

## LimelightLib 2.0.0-beta9-alpha7

- Pure NetworkTables. With no camera, `getRobotPose` returns `new Pose3d()`. Safe in sim.
