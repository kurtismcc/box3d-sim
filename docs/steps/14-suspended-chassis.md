# Step 14: Suspended chassis and log re-enactment

Built after v0, as a record rather than a plan. Library and robot repo.

## Chassis over BUMPs (library)

- `SuspensionConfig(stiffness, damping, groundClearance, rollMoi, pitchMoi)`, set with
  `SwerveDriveConfig.withSuspension`. Without it the chassis is planar, as in step 06.
- The body is free in six DOF with gravity. It rests at z = −mg / (n·k), each wheel pressed in by its share.
- Each wheel is a disc in its steering plane. Seven rays at −0.75 … 0.75 of the wheel radius along the heading are
  cast down the body's −z axis against `FIELD`, from 0.1 m above the axle. A sample at offset s that hits at
  fraction f is pressed in by (1 − f)·(0.1 + √(r² − s²)). The deepest sample is the contact: its hit point and surface
  normal.
- Normal load N = max(0, k·δ − c·(v_contact · n)). The step 06 tire model runs unchanged in the contact plane:
  longitudinal axis = heading projected onto that plane, lateral = n × longitudinal. N·n plus the tire forces are
  applied at the contact point, so load transfer, pitch under acceleration and landings come out of the contact.
- The skirt under the bumpers becomes the frame's underside, from `groundClearance` to the bumper bottom, with frame
  friction. BUMPs are `FIELD`, so robots collide with them.
- The 2026-Offseason values (`SimConstants`): k 70 kN/m (1.75 mm static deflection), c 1000 N·s/m (damping ratio
  about 0.5), 1 in of clearance, roll and pitch MOI 4.6 kg·m² (a uniform 0.9 × 0.9 × 0.55 m box). All ASSUMPTION.

Measured with the test chassis (`BumpCrossingTest`, 6 V on every drive motor):

| Lane | Time 3.2 → 6.0 m | Pitch | Wheel distance / ground distance |
|---|---|---|---|
| BUMP | 1.33 s | 15.9° nose up, 15.5° nose down | 1.31 |
| TRENCH | 1.09 s | under 2° | 1.08 |

On the ridge the belly slides on the HDPE and all four wheels are off the ground for about 0.25 s; they spin up, so
odometry over-reads. At 2–3 V the robot high-centers and stays there. A 27.5 in wheelbase straddling the ridge needs
about 4 in of clearance to stay off it; any clearance allowed under 2.5 in bumpers touches.

## Shooter exit check (library)

`ShooterSim.launch` casts a ray along the launch line from 0.15 m inside the chassis to one radius past the exit
point. If it hits `FIELD` geometry, the launch is refused and the piece stays staged; `exitBlocked()` reports it.
Under a TRENCH arm the robot top is 1.7 cm below the arm, so the shooter can't fire. Before this check, pieces
spawned inside the arm were pushed down into the robot and pinned a suspended chassis to the carpet at about 4.5 kN.

`RebuiltMatch.setFirstInactive` takes FMS game data ("R"/"B") for the current match.

## Robot repo

- `RobotSim` feeds the Pigeon 2 pitch, roll and body-frame angular velocities, and logs
  `FieldSimulation/Drive/NormalLoadsN` and `FieldSimulation/Robot/ExitBlocked`.
- `-PsimMode=reenact -PsimLog=<.wpilog> [-PsimAuto=<name>]` (`LogReenactor`): headless lockstep. Each loop writes the
  log's `/DriverStation/*` values into HAL (the mapping of AdvantageKit's `LoggedDriverStation.replayFromLog`, which
  is package-private) and the robot code drives the physics sim. It starts 2 s before the first enable and stops 1 s
  after the last disable. A log without FMS runs in free play. The robot starts at the logged `Drive/Pose`.
- DataLogReader timestamps in WPILib 2027 alpha 7 come back in ns while the file holds µs; the reader infers the
  unit from AdvantageKit's `/Timestamp` entry, one per 20 ms loop. DataLogReader throws on a truncated last record
  (a robot powered off mid-write), so reading stops at the last complete one.
- Match odometry without vision starts at (0, 0). When the logged start pose is off the field the robot starts at
  the alliance start pose, and logged poses are mapped through that offset; a jump of more than 0.3 m or 20° in one
  loop re-anchors.
- The run is logged to `logs/<name>_reenact.wpilog` with `Reenact/LoggedPose`, `PoseErrorM` and `HeadingErrorDeg`.
  `build/reenact-results.json` has the RMS, max, final and AUTO-end pose error and the path-length ratio.

Re-enacting `logs/akit_26-10-26_21-59-12.wpilog` (an interactive session on the v0 physics, with `Shoot at Hub`):
AUTO matches to under 1 mm, and teleop tracks within 0.15 m for its first 20 s. After the robot scrapes the
guardrail (t ≈ 45 s) runs diverge open loop. Two identical runs gave whole-match RMS errors of 1.9 and 3.6 m and
path ratios of 0.87 and 0.98, because REV's unseeded noise sends them different ways from there. Whole-match error
isn't a stable metric; the re-sync item under "Beyond v0" in the plan is the fix.

Re-enacting SoCal Showdown q10 (`akit_26-10-10_11-27-48_cael_q10.wpilog`, red) exercised a real match log: a truncated
last record, unseeded odometry, FMS game data. AUTO ends 0.35 m short of the logged pose; in teleop the same 0.3 s
full-rate rotation command turned the real robot about 8° and the sim robot 35°, and the sim robot drove 1.47× the
logged path. These numbers aren't calibration data: the code the robot ran at the event can't be verified to match
this branch, and the match had five other robots the sim doesn't.
