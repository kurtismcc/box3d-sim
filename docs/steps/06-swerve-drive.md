# Step 06: Swerve drivetrain and gyro

**Goal:** a swerve chassis that lives in the Box3D world, is driven by per-module drive and steer voltages, and
produces wheel, steer and gyro readings for the robot's simulated sensors.

**Depends on:** steps 04, 05.

Package `box3dsim.drive`.

## What changes from maple-sim

| maple-sim | box3d-sim v0 |
|---|---|
| Wheel speed = ground speed unless skidding; skidding blends 50/50 | Each wheel has its own spin state; force comes from a slip-velocity tire curve with a friction circle |
| Traction cap μ·mg/4, constant | Normal load per module from static weight plus longitudinal and lateral load transfer |
| Extra chassis friction force and torque (hand-tuned gains) | None. Sideways resistance and steering scrub fall out of the lateral tire force |
| Linear/angular damping 1.4 on the chassis | No damping. Losses come from tires and motors |
| 2D dyn4j rectangle | 3D Box3D body (bumper box + superstructure box), planar motion locks |

## Chassis body

- Dynamic body, `MotionLocks.PLANAR` (z translation, roll and pitch locked), `gravityScale = 0`, `bullet = true`
  (fast robot vs. 15 cm balls), sleep disabled, layer `ROBOT`.
- Body origin is on the carpet (z = 0) under the robot center. The chassis never touches the carpet: weight is
  carried implicitly by the z lock, and all traction comes from the tire model below. That avoids fighting the
  contact solver for wheel friction.
- Shapes, all relative to the body origin:
  1. **Bumper:** `Box` half-extents (`bumperLengthX/2`, `bumperWidthY/2`, `bumperHeight/2`), centered at
     z = `bumperBottom + bumperHeight/2`. Material friction 0.5, restitution 0.1, rolling resistance 0. Defaults:
     `bumperBottom` 0.0635 m (2.5 in, bottom of the BUMPER ZONE), `bumperHeight` 0.0825 m (to 5.75 in).
  2. **Under-bumper skirt:** `Box` half-extents (`bumperLengthX/2 − 0.01`, `bumperWidthY/2 − 0.01`,
     `bumperBottom/2 − 0.005`), centered at z = `bumperBottom/2 + 0.005`. Same material. It stops balls (15 cm
     diameter) from slipping under the bumper. Real robots have a frame rail there.
  3. **Superstructure:** `Box` inset `frameInset` (default 0.08 m) from the bumper on each side, from the bumper
     top to `robotHeight`. Material friction 0.3, restitution 0.2.
- Mass properties after adding shapes: `setMassProperties(body, mass, (0,0,cgHeight), (Ixx, Iyy, yawMoi))`. Ixx and
  Iyy don't matter (locked); set them to `yawMoi`.
- Shape density: use any positive value (1.0). The mass is overridden.

## Config

```java
public record SwerveModuleConfig(
        double x, double y,                // module position on the robot, m (robot frame: +x forward, +y left)
        DcMotor driveMotor, double driveGearing, double wheelRadius,
        double wheelMoi,                   // kg·m² at the wheel axle, including the reflected drive rotor and gears
        double driveFrictionTorque,        // N·m at the wheel
        DcMotor steerMotor, double steerGearing,
        double steerMoi,                   // kg·m² about the steer axis
        double steerFrictionTorque) {}     // N·m at the steer axis

public record TireConfig(double staticMu, double kineticMu, double slipVelocity, double slipDecayVelocity) {
    public static TireConfig defaults();   // 1.2, 1.0, 0.08 m/s, 0.5 m/s
}

public record GyroConfig(double driftDegPer30sMotionless, double velocityNoiseFraction,
                         double impactAccelThreshold, double impactDriftRadPerMultiple, long seed) {
    public static GyroConfig ideal();      // 0, 0, ∞, 0, 0
    public static GyroConfig pigeon2();    // 0.5, 0.02, 500 rad/s², 1° per multiple, seed 4470 (maple-sim values)
}

public record SwerveDriveConfig(
        List<SwerveModuleConfig> modules,
        double mass, double yawMoi, double cgHeight,
        double bumperLengthX, double bumperWidthY, double bumperBottom, double bumperHeight,
        double frameInset, double robotHeight,
        TireConfig tire, GyroConfig gyro, boolean loadTransfer) {}
```

`SwerveDriveConfig` validates in its compact constructor: at least 2 modules, mass in (10, 80) kg, positive sizes.

## Per-module state

- Steer: a `LumpedMechanism` (step 05) with
  `Config.of(steerMotor, steerGearing, steerMoi).withFriction(steerFrictionTorque, 0)`. Its position is the module
  azimuth θ_s (rad, continuous, CCW positive, 0 = wheel rolling along robot +x).
- Drive: wheel spin ω_w (rad/s) and wheel angle φ_w (rad, continuous). These are integrated by the tire loop below,
  not by `LumpedMechanism`, because the wheel couples to the ground.
- Inputs per step: `driveVolts`, `driveCoast`, `steerVolts`, `steerCoast`. Positive drive volts spin the wheel to
  roll toward module heading θ_s.
- Outputs: wheel position φ_w, wheel velocity ω_w, drive applied volts, drive stator current, drive supply current,
  steer position θ_s and velocity, steer currents.

## Physics step

`SwerveDriveSim` has two public methods: `prePhysics(double dt)` computes and applies forces, and
`postPhysics(double dt)` reads back the chassis state. Tests in this step call them around `world.step(dt, 4)`
directly. Step 07 introduces `ArenaParticipant` and makes `SwerveDriveSim` implement it by delegating to these two
methods.

### `prePhysics(dt)`

First call `ensureMass()`: if `|world.mass(body) − config.mass| > 1e-6`, call `setMassProperties` again. Shapes
added later (step 10's intake sensor) use `updateBodyMass = false`, but this guard costs one FFM call and makes
the drivetrain independent of that rule.

Read once from the world: chassis pose (x, y, yaw ψ) and velocities v (vx, vy) and Ω (yaw rate).

1. **Normal loads.** Let `n` be the module count and `g = 9.81`.
   - If `loadTransfer` is on, let `a_x, a_y` be the chassis acceleration in the **robot frame**. It is the
     finite difference of robot-frame velocity across the previous step, low-pass filtered with τ = 0.05 s, and
     starts at 0. Then
     `N_i = m·g/n − m·h·(a_x·x_i / (n·mean(x²)) + a_y·y_i / (n·mean(y²)))`, clamped to `[0.05·m·g/n, 2·m·g/n]`.
     Here `mean(x²)` averages over modules.
   - If it is off, `N_i = m·g/n`.
2. **Steer.** `steer.setInput(steerVolts, steerCoast); steer.step(dt)`.
3. **Contact-point velocity** for each module, in the field frame:
   `p_i = chassisPose.transformPoint((x_i, y_i, 0))` and `v_i = v + Ω ẑ × (p_i − c)` where c is the chassis
   position. Equivalently use `world.pointVelocity(body, p_i)`, but compute it in Java once from v and Ω to save FFM
   calls. Module heading in the field frame: `h = (cos(ψ+θ_s), sin(ψ+θ_s))`, lateral `l = (−h.y, h.x)`. Then
   `u = v_i·h` (rolling direction speed) and `w = v_i·l` (side slip speed).
4. **Wheel micro-steps.** Run K = 10 micro-steps of `δ = dt/K`, holding u and w constant:
   - **Motor:** `a = G·Kt/R`, `b = G²·Kt/(Kv·R)`. Voltage V = `clamp(driveVolts, ±battery)`. In coast at zero volts,
     the motor torque is 0. Otherwise `τ_m = a·V − b·ω_w`. Use the same semi-implicit form as `LumpedMechanism` if you
     factor it out, but explicit is fine here because δ is 0.4 ms.
   - **Slip vector of the patch over the ground:** `s = (u − ω_w·r, w)`, σ = |s|.
   - **Friction coefficient:** `μ(σ) = μ_k + (μ_s − μ_k)·exp(−σ / slipDecayVelocity)`.
   - **Ground force on the wheel** in (h, l) components: `F = −μ(σ)·N_i·tanh(σ / slipVelocity)·(s / σ)`; F = 0 if
     σ < 1e-9.
   - **Wheel spin:** `J·dω_w/dt = τ_m − r·F_h − sign(ω_w)·τ_friction`. F_h is the h component of F. When the wheel
     spins faster than the ground (ω_w·r > u), F_h > 0. It pushes the robot forward and its reaction `−r·F_h`
     decelerates the wheel. Check the sign with test 2 below. Apply Coulomb friction as in `LumpedMechanism`: if
     |ω_w| would cross zero from friction alone, clamp to 0.
   - `φ_w += ω_w·δ`.
   - Accumulate `F_avg += F / K`, and the stator current average (for logging and battery).
5. **Apply to chassis:** field-frame force `F_field = F_avg.h·h + F_avg.l·l` at the module's ground point `p_i`
   (z = 0). The z lock removes the pitching moment; that's why load transfer is computed analytically in step 1.
6. Record drive applied volts, stator current, and supply current
   `I_s = I_stator·V/V_battery` (0 in coast).

**Stability margins** (with defaults: m = 50 kg, N ≈ 123 N per module, μ_s/slipVelocity·N ≈ 1845 N·s/m):

- Chassis: 4·1845/50·dt = 0.59 at dt = 4 ms (< 1).
- Wheel: 1845·r²/J·δ ≈ 0.53 at δ = 0.4 ms with J = 0.002 kg·m².

If a test shows oscillation, raise K. Do not lower `slipVelocity` below 0.05.

### `postPhysics(dt)`

1. Read pose and velocities.
2. Update the gyro.
3. Update the acceleration filter for load transfer.

## Gyro

`GyroSim`: the reading starts at 0 and accumulates the body's **actual yaw change** plus errors. All randomness
comes from `new Random(seed)`, so runs are reproducible.

- Each `postPhysics`, compute `dψ = wrapToPi(ψ_body − ψ_body_prev)` (true change this step; Box3D integrates
  within its substeps, so don't integrate Ω yourself) and `Ω_true = dψ/dt`.
- `measuredRate = Ω_true + N(0, velocityNoiseFraction·|Ω_true|)`, then `yaw += measuredRate·dt`. With the ideal
  config this is exactly the unwrapped body yaw.
- **Impact drift:** `α = (Ω_true − Ω_prev)/dt`. If |α| > threshold, add `−sign(α)·(|α|/threshold)·impactDriftRadPerMultiple`.
- **Motionless drift:** add `N(0, driftDegPer30sMotionless/30·dt)` degrees, converted to rad.

API: `yawRad()` (continuous), `yawRateRadPerSec()` (measured), `setYaw(rad)`.

Robot code reads yaw through `Pigeon2SimState.setRawYaw(degrees)`, wired in step 11.

## Public API

```java
public final class SwerveDriveSim implements ElectricalLoad {     // + ArenaParticipant in step 07
    public SwerveDriveSim(SimWorld world, Battery battery, SwerveDriveConfig config, Pose3 startPose);
    public void prePhysics(double dt);
    public void postPhysics(double dt);
    public int moduleCount();
    public void setModuleInputs(int i, double driveVolts, boolean driveCoast, double steerVolts, boolean steerCoast);
    public double wheelPositionRad(int i); public double wheelVelocityRadPerSec(int i);
    public double steerPositionRad(int i); public double steerVelocityRadPerSec(int i);
    public double driveAppliedVolts(int i); public double driveStatorCurrent(int i);
    public double steerAppliedVolts(int i); public double steerStatorCurrent(int i);
    public Pose3 pose();                       // true pose (z = 0)
    public Vec3 fieldVelocity(); public double yawRate();
    public GyroSim gyro();
    public void teleport(double x, double y, double yaw);   // zero velocities, zero wheel spin, gyro unchanged
    public BodyHandle body();
    public SwerveDriveConfig config();
    /** Re-applies the configured mass if something (a shape added later) reset it. Called at the top of prePhysics. */
    void ensureMass();
    LumpedMechanism steer(int i);              // package-private, for tests
    double normalLoad(int i);                  // package-private, for tests; N from the last prePhysics
    @Override public double supplyCurrent();   // sum of drive and steer supply currents
}
```

## Test preset

Robot presets live in the robot repo (step 11), not in the library. Tests here use
`src/test/java/box3dsim/drive/TestConfigs.maxSwerve()`. It holds the same numbers step 11 uses for the
2026-Offseason robot:

| Field | Value | Source |
|---|---|---|
| modules | FL (+0.3493, +0.3493), FR (+0.3493, −0.3493), BL (−0.3493, +0.3493), BR (−0.3493, −0.3493) | code `Drive.Constants`, 27.5 in |
| drive | `DcMotor.neoVortex(1)`, gearing 3.625, wheel radius 0.0381 m | `DRIVING_FACTOR` (what the encoders assume) |
| wheelMoi | 0.002 kg·m² | assumption: rotor ~7e-5 × 3.625² + 3 in wheel and gears |
| driveFrictionTorque | 0.15 N·m | assumption, about 0.1 V of friction voltage |
| steer | `DcMotor.neo550(1)`, gearing 9424/203 = 46.42, steerMoi 0.004 kg·m², friction 0.1 N·m | MAXSwerve |
| mass, yawMoi | 50 kg, 6.883 kg·m² | PathPlanner `settings.json` |
| cgHeight | 0.20 m | assumption |
| bumper | 0.9 × 0.9 m, bottom 0.0635, height 0.0825 | `settings.json`, game manual R405 |
| robotHeight, frameInset | 0.55 m, 0.08 m | assumption. 0.55 < the 0.565 m TRENCH clearance; confirm with Kurtis |
| tire | `TireConfig.defaults()` (μ_s 1.2 matches settings.json wheelCOF) | |
| gyro | `GyroConfig.pigeon2()` | |
| loadTransfer | true | |

## Tests: `src/test/java/box3dsim/drive/SwerveDriveSimTest.java`

The world is a static ground plus nothing else. dt = 0.004, Box3D substeps 4. The battery is `Battery.defaults()`
updated each step with the drive's supply current. Gyro is ideal unless stated. Steer stays at its initial angle
(0) unless stated.

1. **Straight line, top speed:** all modules at 12 V drive, steer 0 V. After 3 s, vx is within 5% of
   `ω_free·r/G` scaled by the loaded battery voltage. Compute the expectation from the motor model at the
   measured battery voltage, roughly 7 m/s. |vy| < 0.01 and |Ω| < 0.01.
2. **Force sign:** at t = 0.1 s of the same run, vx > 0 and every wheel has ω_w·r > u (driving slip).
3. **Acceleration is traction- or current-limited:** peak acceleration in the first 0.2 s ≤ μ_s·g·1.05 ≈ 12.4 m/s².
4. **Coast-down vs brake:** from 3 m/s, all drive volts 0.
   - Coast: the robot rolls more than 2 m.
   - Brake: it stops within 1.0 m and in less than a third of the coast distance. The motor braking gain is about
     G²·Kt/(Kv·R·r²) ≈ 45 N·s/m per module, which gives τ ≈ 0.3 s and roughly 0.8 m.
5. **Spin in place:** set steer angles to the tangent directions (θ_s = atan2(x_i, −y_i)) with `steer(i).setState`,
   hold steer volts 0, then drive 6 V. After 2 s, |vx|, |vy| < 0.05 and Ω > 0. The ideal gyro yaw equals the
   **unwrapped** body yaw within 1e-6.
6. **Sideways resistance:** teleport the robot with vy = 2 m/s (`world.setVelocity`) and wheels pointing along +x,
   drive in brake at 0 V. vy decays to < 0.05 within 0.3 s (lateral tire force).
7. **Load transfer:** during test 1's first 0.2 s, the rear modules (x < 0) have a larger `normalLoad(i)` than the
   front.
8. **Wall:** add a static box wall whose −x face is at x = 2 (layer `FIELD`, box x [2, 2.2]). Driving at 12 V into
   it stops the robot with its **body center** at x ≈ 2 − bumperLengthX/2 (± 0.02), which puts the bumper face
   against the wall. No tunneling, and the body z stays 0.

## Acceptance

- `.\gradlew.bat test` passes.
- A 30 s straight-drive test at dt = 0.004 runs in under 1 s of CPU time. Report the time; don't optimize unless it
  fails.

Commit: `Add swerve drivetrain with tire slip and load transfer`.
