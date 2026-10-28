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

v0 plays 2026 REBUILT with the 2026-Offseason robot (`D:\Projects\2026-Offseason`, branch `box3d-sim`; see its
`SIMULATION.md`), interactively, headless, or re-enacting a logged match. The plan is in [docs/plan.md](docs/plan.md);
the suspended chassis is [step 14](docs/steps/14-suspended-chassis.md).

## v0 limitations

- **Wheel contact.** With a `SuspensionConfig` the chassis is free in six DOF. Each wheel is a disc sampled by
  seven rays, pressed into a stiff spring-damper, so it drives over BUMPs, pitches and lands. There is no tread or
  carpet compliance beyond the slip curve, and wheels can't droop. A 27.5 in wheelbase with 1 in of clearance
  high-centers on a BUMP's ridge, so a slow crawl gets stuck there. Without a `SuspensionConfig` the chassis is
  planar, as in v0, and BUMPs stop it.
- **Shooter exit check** is a single ray along the launch line. A blocked exit (under a TRENCH arm) keeps the piece in
  the robot; one that only grazes geometry is spawned and pushed out by the solver.
- **Intake** has no physical roller collider: FUEL in the deployed zone is taken or ignored.
- **Storage** is counted, not simulated. There are no jams or ball-to-ball contact inside the robot.
- **HUB** has a square rim instead of the hex, no internal funnel, and exits that are spawn points with random speeds.
- **OUTPOST chute** isn't physical; released FUEL spawns at the chute mouth.
- **One robot.** No opponents or partners, no towers or climbing, no vision.
- **Robot constants.** Shooter, intake and storage constants are assumptions until calibrated against the real robot.
- **Rolling resistance** is applied as a force by `GamePieces`, because Box3D's own stops sliding spheres from
  spinning up.
- **Windows x64 natives only.**

## Constraints

- Targets WPILib 2027 (`org.wpilib.*` packages, SystemCore).
- Box3D is native code. Desktop simulation and CI runners need platform binaries.
