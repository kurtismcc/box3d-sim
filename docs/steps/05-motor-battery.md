# Step 05: DC motor, battery and lumped mechanisms

**Goal:** the electrical and single-degree-of-freedom mechanical models every mechanism uses: DC motor
constants, a battery with sag, and a lumped motor-plus-gearbox-plus-inertia mechanism with friction, hard limits
and an optional current limit.

**Depends on:** step 01 (no physics world needed).

These replace maple-sim's `DCMotor` (from WPILib), `SimMotorConfigs`, `SimMotorState`, `MapleMotorSim` and
`SimulatedBattery`. There are three behavior changes:

- The back-EMF term is integrated semi-implicitly, so tiny inertias stay stable.
- Coast and brake neutral modes are modeled.
- The battery uses a first-order filter rather than a 50-sample moving average.

Package `box3dsim.motor`.

## `DcMotor`

```java
public record DcMotor(String name, double nominalVoltage, double stallTorque, double stallCurrent,
                      double freeCurrent, double freeSpeedRadPerSec, int motorCount) {
    public double resistance();     // R  = nominalVoltage / stallCurrent
    public double kv();             // Kv = freeSpeed / (nominalVoltage − R·freeCurrent)   [rad/s per V]
    public double kt();             // Kt = stallTorque / stallCurrent                    [N·m per A]
    /** Stator current at rotor speed ω and applied voltage V: (V − ω/Kv) / R. */
    public double current(double rotorSpeedRadPerSec, double volts);
    public double torque(double currentAmps);                  // Kt·I
    public double voltage(double torque, double rotorSpeedRadPerSec); // torque/Kt·R + ω/Kv

    public static DcMotor neoVortex(int count);
    public static DcMotor neo(int count);
    public static DcMotor neo550(int count);
    public static DcMotor krakenX60(int count);   // for later robots; not used by 2026-Offseason
}
```

For `count` motors geared together, stall torque, stall current and free current are multiplied by `count`;
free speed and voltage stay the same. This matches WPILib `DCMotor.getNeoVortex(n)` and similar.

| Motor | V | Stall torque (N·m) | Stall current (A) | Free current (A) | Free speed (rpm) |
|---|---|---|---|---|---|
| NEO Vortex | 12 | 3.60 | 211 | 3.6 | 6784 |
| NEO | 12 | 2.6 | 105 | 1.8 | 5676 |
| NEO 550 | 12 | 0.97 | 100 | 1.4 | 11000 |
| Kraken X60 | 12 | 7.09 | 366 | 2 | 6000 |

These are WPILib's `DCMotor` factory values. Convert rpm to rad/s with `rpm · 2π / 60`.

## `Battery`

```java
public final class Battery {
    public Battery(double openCircuitVolts, double internalResistanceOhms, double brownoutVolts, double filterTimeConstant);
    public static Battery defaults();                 // 12.6 V, 0.020 Ω, 6.8 V, 0.04 s
    /** Call once per physics step with the total supply current drawn during that step. */
    public void update(double totalSupplyCurrentAmps, double dt);
    public double voltage();                          // loaded terminal voltage, never below brownoutVolts
    public double filteredCurrent();
    public boolean isBrownedOut();                    // true when the clamp is active
    public void reset();                              // voltage = openCircuitVolts, filtered current = 0
}
```

Model: `I_f += α·(I − I_f)` with `α = dt / (τ + dt)`, then `V = max(brownout, V_oc − R·I_f)`.

The filter is not cosmetic. Motor currents depend on battery voltage, and battery voltage depends on the
currents, which closes a loop through one step of delay. With about 10 motors the loop gain is above 1. Without
the filter, the voltage oscillates from one step to the next.

## `ElectricalLoad`

```java
public interface ElectricalLoad {
    /** Supply (battery-side) current in amps for the step just simulated. May be negative when regenerating. */
    double supplyCurrent();
}
```

`LumpedMechanism` and the swerve modules (step 06) implement it. The arena (step 07) sums all loads into
`Battery.update` after each physics step.

Supply current from stator current: `I_supply = I_stator · V_applied / V_battery`, which is conservation of power
through the motor controller. If `V_battery < 1e-6`, use 0.

## `LumpedMechanism`

One rigid degree of freedom: motor(s) through a reduction into a load inertia. Use it for the flywheel, hood,
intake pivot, rollers and swerve steering.

```java
public final class LumpedMechanism implements ElectricalLoad {
    public record Config(DcMotor motor,
                         double gearing,            // rotor rotations per mechanism rotation (> 0)
                         double moi,                // kg·m² at the mechanism, including the reflected rotor
                         double frictionTorque,     // Coulomb, N·m at the mechanism (≥ 0)
                         double viscousDamping,     // N·m·s/rad at the mechanism (≥ 0)
                         double minPosition,        // mechanism rad; NEGATIVE_INFINITY for none
                         double maxPosition,
                         double statorCurrentLimit) // A; POSITIVE_INFINITY for none
    {
        public static Config of(DcMotor motor, double gearing, double moi); // no friction, no limits
        public Config withFriction(double coulomb, double viscous);
        public Config withLimits(double min, double max);
        public Config withCurrentLimit(double amps);
    }

    public LumpedMechanism(Config config, Battery battery);

    /** Input for the next step(). volts is the controller's requested output; clamped to ±battery voltage. */
    public void setInput(double volts, boolean coastAtZero);
    /** Extra torque at the mechanism for the next step only (gravity, game piece loads). Accumulates. */
    public void addExternalTorque(double newtonMeters);
    /** Removes energy instantly: sets |ω| to sqrt(max(0, ω² − 2E/J)). For flywheel shots. */
    public void removeEnergy(double joules);

    public void step(double dt);

    public double position();        // mechanism rad
    public double velocity();        // mechanism rad/s
    public double rotorPosition();   // position · gearing
    public double rotorVelocity();   // velocity · gearing
    public double appliedVolts();
    public double statorCurrent();
    @Override public double supplyCurrent();
    public void setState(double position, double velocity);
    public Config config();
}
```

### `step(dt)` algorithm

Let G = gearing, J = moi, R = resistance, Kv = kv, Kt = kt, and ω the mechanism velocity.

1. `V = clamp(requestedVolts, −battery.voltage(), +battery.voltage())`.
2. **Coast:** if `coastAtZero && |V| < 1e-9`, the windings are open. Electrical torque and current are 0. Skip step 3,
   and run step 4 with `a = b = 0`. That keeps `τ_ext` and viscous damping.
3. **Current limit:** the stator current at the current speed is `I = (V − G·ω/Kv)/R`. If
   `|I| > statorCurrentLimit`, recompute the voltage that gives exactly the limit:
   `V = sign(I)·limit·R + G·ω/Kv`, then clamp again to ±battery.
4. **Semi-implicit electrical step:** motor torque at the mechanism is
   `τ_e = G·Kt·(V − G·ω/Kv)/R = a·V − b·ω`, where `a = G·Kt/R` and `b = G²·Kt/(Kv·R)`. Solve
   `ω' = (ω + dt/J·(a·V + τ_ext)) / (1 + dt·(b + viscousDamping)/J)`.
5. **Coulomb friction:** `Δ = dt·frictionTorque/J`. If `|ω'| ≤ Δ`, set `ω' = 0`; otherwise `ω' −= sign(ω')·Δ`.
6. **Position:** `θ' = θ + ω'·dt`.
7. **Hard limits:** if `θ' < min`, set `θ' = min` and `ω' = max(ω', 0)`. If `θ' > max`, set `θ' = max` and
   `ω' = min(ω', 0)`.
8. **Store outputs:** `appliedVolts = V` (0 in coast), `statorCurrent = (V − G·ω'/Kv)/R` (0 in coast), and the
   supply current from the formula above. Clear the external torque accumulator.

### `removeEnergy(E)`

`ω_new = sign(ω)·sqrt(max(0, ω² − 2E/J))`.

## Tests: `src/test/java/box3dsim/motor/`

`MotorModelTest`:

1. **Vortex constants:** `R ≈ 0.05687 Ω`, `Kt ≈ 0.01706 N·m/A`, `Kv ≈ 710.4 / (12 − 0.05687·3.6) ≈ 60.2 rad/s per V`.
2. **Free speed:** a Vortex on a 1:1 mechanism with J = 1e-3, 12 V, no friction, battery replaced by a stiff source
   (`new Battery(12, 0, 0, 1e-9)`). After 2 s at dt = 0.004 the rotor speed is within 1% of
   `Kv·(12 − R·0)`. That is the no-load speed of the model, which is slightly above the datasheet free speed
   because there is no free current.
3. **Time constant:** the same setup reaches 63.2% of final speed at `t ≈ J·R·Kv/Kt` within 5%.
4. **Stability with tiny inertia:** a NEO 550 at gearing 50, J = 1e-6, dt = 0.004, 12 V, stiff source. The
   mechanism speed never exceeds the model's no-load speed `Kv·12/50 × 1.001` and never goes negative. The
   model's no-load speed is about 1.4% above the datasheet free speed.
5. **Coast vs brake:** test 2's setup plus friction 0.01 N·m. Spin to no-load speed, then set 0 V.
   - Coast: the speed decays linearly at `0.01/J` rad/s².
   - Brake (`coastAtZero = false`): it decays exponentially with τ = J·R·Kv/Kt ≈ 0.20 s. **At 0.2 s** the brake speed
     is < 0.4× the start speed and less than half the coast speed.
6. **Current limit:** a Vortex at stall (J huge, 1e3) with 12 V and a 40 A limit gives a stator current of 40 A ± 0.5.
7. **Hard limits:** with max = 1.0 rad, driving at 12 V settles at θ = 1.0 and ω = 0.
8. **removeEnergy:** at ω = 300 rad/s and J = 0.002, removing 10 J gives ω = sqrt(90000 − 10000).

`BatteryTest`:

1. With 0 A, the voltage stays 12.6.
2. With a constant 100 A for 1 s at dt = 0.004, the voltage converges to 12.6 − 2.0 = 10.6 ± 0.01.
3. With 1000 A, the voltage clamps at 6.8 and `isBrownedOut()` is true.

## Acceptance

- `.\gradlew.bat test` passes.
- No WPILib imports in `box3dsim.motor`.

Commit: `Add DC motor, battery and lumped mechanism models`.
