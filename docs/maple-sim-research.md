# maple-sim: what it models

Notes from reading the maple-sim source to decide what to keep and what to replace with Box3D.

- Repo: https://github.com/Shenzhen-Robotics-Alliance/maple-sim
- Commit read: `1b1e8ea7278de01d40a5ea14dffbaaf13381512a` (main, 2026-07-07)
- License: MIT (Team 5516 "Iron Maple"). Porting code is allowed with attribution.
- Size: about 6,500 lines of Java. Physics engine is dyn4j 5.0.2 (2D).

Paths below are relative to `project/src/main/java/org/ironmaple/simulation/`.

## Summary

maple-sim does **not** do detailed mechanism physics. It has no shaft compliance, backlash, belt stretch,
or game-piece compression. Its value is the layer between robot code and the physics world: the motor and
battery model, the swerve force model, simulated sensors, and season-specific field and scoring rules. The
physics world itself is a 2D rigid-body world, and flying game pieces are analytic parabolas that never
collide with anything.

## Motors: `motorsims/`

`MapleMotorSim.update` and `SimMotorState.step`:

- One lumped inertia per mechanism (`loadMOI`). Voltage gives current from the motor model, current gives
  torque, and torque divided by inertia updates the speed.
- Semi-implicit Euler: velocity first, then position from the new velocity.
- Friction is one constant Coulomb torque. If friction would reverse the sign of velocity within a step,
  velocity is set to zero.
- Hard limits clamp position and zero velocity. They don't model impact or bounce.
- Applied voltage is clamped to the simulated battery voltage. Current limits live in `SimMotorConfigs`.
- Equivalent in depth to WPILib's `DCMotorSim`. There's no multi-body drivetrain, gear backlash or
  compliance.

`SimulatedBattery`:

- Sums supply current from every registered motor and other loads, smooths it with a 50-sample moving
  average, and calls WPILib `BatterySim.calculateLoadedBatteryVoltage(13.5 V, 0.02 Ω, I)`.
- Clamps to brownout voltage and writes it to `RoboRioSim.setVInVoltage`.
- Supply current is derived from stator current by conservation of power: `I_supply = I_stator · V_applied / V_battery`.
- Useful: battery sag feeds back into every motor, so brownouts change how the robot drives.

## Swerve: `drivesims/`

`SwerveModuleSimulation.getPropellingForce`:

- Wheel force = drive motor torque ÷ wheel radius.
- Traction is a hard cap: `μ · normal force on that module`. Beyond the cap the module is treated as
  skidding.
- Not skidding: the wheel speed is the chassis ground velocity projected onto the wheel direction, with no
  slip.
- Skidding: the wheel speed is blended 50/50 between the ground-projected speed and the motor's equilibrium
  speed at the capped torque. That blend is a heuristic, not a slip model.
- No tire slip curve, no load transfer under acceleration, and no scrub torque from steering.

`SwerveDriveSimulation`:

- The chassis is a dyn4j `Body`: one rectangle including bumpers.
- Module forces are applied at each module's position.
- A separate total friction force and torque resists sideways sliding.
- Linear and angular damping are both 1.4 (`AbstractDriveTrainSimulation` sets 0.1, and the swerve
  subclass overrides it).
- Steer motors use the same `MapleMotorSim` model.
- The relative steer encoder starts at a random offset of ±15 rad to mimic an unhomed encoder.
- `GyroSimulation` adds drift and noise. Encoder readings are cached per sub-tick for high-frequency
  odometry.

## World: `SimulatedArena`

- A dyn4j `World` with **gravity set to zero**: a top-down 2D plane.
- Default timing is 5 sub-ticks per 20 ms robot period (4 ms steps). Each sub-tick calls
  `physicsWorld.step(1, dt)`.
- Field walls and elements are 2D segments and rectangles (`FieldMap.addBorderLine`, `addRectangularObstacle`).
- Game pieces on the floor (`GamePieceOnFieldSimulation`) are 2D dyn4j bodies with damping, so robots can
  push them.
- Static singleton timing state, guarded by a lock.

## Game pieces in flight: `gamepieces/GamePieceProjectile`

- **Not a physics body.** Position is a closed-form parabola from launch position, speed and angle.
- `GRAVITY = 11` m/s². The source comment says 9.8 looked wrong, likely because air drag is ignored.
- No drag, spin or Magnus lift.
- No collisions while airborne: no bouncing off rims, field elements, robots or other pieces.
- Hitting a target is checked by sampling the parabola for a point within a tolerance box (default
  ±0.2 m per axis) of a target point.
- On landing it optionally becomes a 2D on-field piece again.

## Intakes: `IntakeSimulation`

- A thin rectangle fixture attached to the chassis on one side.
- A matching game piece that touches the fixture while the intake is running is removed from the world
  and counted, up to a capacity.
- No contact mechanics: no rollers, compression or partial grabs.

## Season content: `seasonspecific/`

- Arenas for 2024 Crescendo, 2025 Reefscape and 2026 Rebuilt, plus an evergreen field.
- Includes field obstacle maps, goals (`Goal`, `RebuiltHub`), outpost and human-player logic, and starting
  piece layouts.
- This is the most labor-intensive part to reproduce, and it gets rewritten every season anyway.

## 2027 compatibility

- Builds with GradleRIO **2026.2.1** and Java 17.
- 219 `import edu.wpi.first.*` lines across 47 files. WPILib 2027 moved to `org.wpilib.*` packages, so the
  library does not compile or link against the 2027 alphas.
- Uses `RoboRioSim` for battery voltage and brownout, which needs a SystemCore equivalent.
- Upstream had no 2027 branch at the commit above. Branches were `main`, `fix-timer`, `gh-pages` and two
  Copilot branches.

## Keep vs. replace

| Area | maple-sim | Plan |
|---|---|---|
| Motor + battery model | Lumped DC motor, Coulomb friction, `BatterySim` sag | Port, then extend (compliance, backlash) where it matters |
| Swerve module forces | Traction cap plus heuristic skid blend | Port, then add a slip curve and load transfer |
| Simulated sensors | Encoder caches, gyro drift | Port |
| Rigid-body world | dyn4j 2D, zero gravity | Replace with Box3D 3D world |
| Flying game pieces | Analytic parabola, g = 11, no collisions | Replace with Box3D bodies: drag, restitution, real contacts |
| Intakes | Contact-to-counter rectangle | Replace with contact volumes or roller bodies |
| Scoring | Tolerance box around a point | Replace with sensor volumes inside goal geometry |
| Season field/rules | Hand-built 2D maps | Rebuild in 3D per season |
