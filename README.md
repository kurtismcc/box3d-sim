# box3d-sim

A physics simulation library for FRC robot code, built on [Box3D](https://box2d.org/).

## Idea

Desktop simulation should be good enough to tune autos, shooters and intakes before the robot exists. It
should also feel like driving the real robot.

[maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) shows what that layer looks like, but
its physics is shallow:

- a 2D world,
- a single-inertia motor model,
- flying game pieces drawn as analytic parabolas that never collide.

See [docs/maple-sim-research.md](docs/maple-sim-research.md).

This project keeps the useful robot-side layer and puts a real 3D world under it:

- **Box3D world.** Chassis, field elements and game pieces are 3D rigid bodies in one world. Balls bounce
  off rims, pile up and get pushed around by robots.
- **More accurate mechanism models.** Start from the DC motor plus battery-sag model, then add what
  changes outcomes: drivetrain compliance and backlash, a tire slip curve with load transfer, and roller
  and intake contact mechanics.
- **Same robot-code seam.** Robot code talks to AdvantageKit-style IO interfaces. The sim versions feed
  motor voltages in and read encoders and gyros out. Robot code never imports this library directly.
- **Swappable backends.** A `SimWorld` interface sits between the mechanism layer and the physics engine.

## Layers

```
Robot IO sim implementations          (robot repo)
   │ volts out / encoder, gyro in
Mechanism layer                       motors, battery, swerve modules, sensors
   │ forces, torques, launch and intake events
SimWorld interface
   └─ Box3D backend                   native, via Java FFM (Java 22+)
Visualization                         poses over NetworkTables → AdvantageScope
```

## Status

v0 in progress. See [docs/plan.md](docs/plan.md).

## Constraints

- Targets WPILib 2027 (`org.wpilib.*` packages, SystemCore).
- Box3D is native code. Desktop simulation and CI runners need platform binaries.
