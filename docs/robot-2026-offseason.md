# 2026-Offseason robot: what the sim must model

Inventory of `D:\Projects\2026-Offseason` (branch `alpha7`, commit `2321eb5` "Final changes before comp"), the
robot used at SoCal Showdown. This is the robot box3d-sim v0 is built against. Paths below are relative to
`src/main/java/frc/robot/` unless noted.

## Build and toolchain

| Item | Value |
|---|---|
| WPILib | GradleRIO `2027.0.0-alpha-7`, `org.wpilib.*` packages, SystemCore target |
| Java | 25 (`sourceCompatibility = VERSION_25`) |
| Gradle wrapper | 9.6.1 |
| Desktop sim | `includeDesktopSupport = false` in `build.gradle`, but it's an unused variable in GradleRIO alpha-7 (only VS Code's "Change Desktop Support" reads it). Desktop JNI is already pulled in. Sim runs with `.\gradlew.bat run` |
| Sim extensions | `wpi.sim.addGui().defaultEnabled = true`, `wpi.sim.addDriverstation()` |
| Team | 4470 (`.wpilib/wpilib_preferences.json`) |

Vendordeps: AdvantageKit `27.0.0-alpha-6`, REVLib `2027.0.0-alpha-8`, Phoenix6 `26.70.0-alpha-2`, PathPlannerLib
`2027.0.0-alpha-4.1`, LimelightLib `2.0.0-beta9-alpha7`, CommandsV2. All JNI deps list `windowsx86-64`.

WPILib 2027 alpha 7 is **not installed** on this machine (only `C:\Users\Public\wpilib\2025`), and the project has
never been built here. The installed AdvantageScope is the 2025 one, which predates the 2026 field.

## Structure

- `Robot extends LoggedRobot` (AdvantageKit). `RobotContainer` builds subsystems behind feature flags in
  `Constants/Operating.java`: drive, gyro, shooter, intake and auto are on; vision, climb, operator and limelight
  are off.
- One gamepad, port 0 (`CommandGamepad`). Operator port 1 is defined but unused.
- IO layer is half-built: `PIDMotorIO` (REV-flavored, takes `SparkLowLevel.ControlType`) with `PIDMotorIOSparkMax`
  and `PIDMotorIOSparkFlex`; `SwerveModuleIO` with `SwerveModuleIONEO`. Only `velocity` is logged in
  `PIDMotorIOInputs`, so log replay is not meaningful.
- The Pigeon2 and Limelight are used directly inside `DriveSubsystem`, with no IO layer.

## Drivetrain: `subsystems/DriveSubsystem.java`, `components/SwerveModule*.java`

Swerve, 4 modules, REV MAXSwerve-style: SparkFlex drive (NEO Vortex), SparkMax turn (NEO 550 assumed) with a
through-bore absolute encoder on the module.

| Item | Code value | Source |
|---|---|---|
| Module positions | ±13.75 in (0.349 m) x and y, FL, FR, BL, BR order | `Constants/Drive.java` `WHEEL_BASE`, `TRACK_WIDTH` = 27.5 in |
| Wheel | 3 in diameter | `ModuleConstants.WHEEL_DIAMETER` |
| Drive reduction used by encoder conversion | 3.625 | `DRIVING_FACTOR = circumference / 3.625` |
| Drive reduction used for free speed / FF | 3.0 (`3.5625 * 16 / 19`) | `DRIVE_MOTOR_REDUCTION` |
| Motor free speed | 6784 rpm (Vortex) | `DRIVE_MOTOR_FREE_RPS` |
| Max speed (teleop scale) | 7.60 m/s × 0.3 multiplier in `drive()` | `MAX_METERS_PER_SECOND` |
| Max rotation | 4π rad/s × 0.5 | `MAX_ANGULAR_SPEED` |
| Angular offsets | FL π/2, FR 0, BL 0, BR π/2 | `*_ANGULAR_OFFSET` |
| CAN IDs | drive FL 11, BL 12, FR 13, BR 14; turn FL 18, BL 16, FR 17, BR 15; Pigeon 8 | `Constants/IDs.java` |
| Bus | all drive and Pigeon on `CAN_S0` | `Constants/CAN.java` |

Spark configs (`Constants/Configs.java`):

- Drive (SparkFlex): brake, smart current limit 40 A, secondary 90 A, FL and BL inverted, closed loop on the
  primary encoder, `p = 0.0001`, `kV = 1 / DRIVE_WHEEL_FREE_MPS / 60` (≈ 0.00185), output range ±1.
- Turn (SparkMax): brake, 20 A / 70 A, absolute encoder inverted, closed loop on the absolute encoder, `p = 3`,
  output ±1, position wrapping on.
- Conversion factors are no longer set in the Spark configs (comments say "set through hardware manager"). The IO
  classes convert in Java instead: setpoints are divided by the factor before going to the Spark, readings are
  multiplied by it. Drive setpoints therefore reach the Spark in motor rotations and RPM. Turn setpoints reach it
  in rotations (`radians / 2π`), so the turn controller works in absolute-encoder rotations.

Behavior:

- `drive(x, y, rot, fieldRelative)` scales sticks, converts with `toRobotRelative(gyro yaw)`, desaturates and
  commands modules. `SwerveModuleIONEO.setDesiredVelocity` adds the chassis offset, optimizes against the raw
  absolute angle, then calls `driveMotor.setVelocity` (kVelocity) and `turnMotor.setSetpoint` (kPosition).
- Odometry: both `SwerveDriveOdometry` (`getOdometry()`, used by PathPlanner) and `SwerveDrivePoseEstimator`
  (logged as `Drive/Pose`) are updated in `periodic()`. Heading comes from `Pigeon2.getYaw()`.
- PathPlanner: `AutoBuilder.configure(getOdometry, resetOdometry, getRobotRelativeVelocities, driveRobotRelative,
  PPHolonomicDriveController(P 0.04 translation, 0.5 rotation), RobotConfig.fromGUISettings(), flip for red)`.
- Logs `Power/BatteryVoltage` from `RobotController.getBatteryVoltage()` and `Match/TimeRemaining` from
  `Timer.getMatchTime()`.

`src/main/deploy/pathplanner/settings.json` disagrees with the code: mass 50 kg, MOI 6.883 kg·m², module
positions ±0.273 m (code: ±0.349 m), drive gearing 4.23 (code: 3.625), wheel radius 0.038 m, bumpers 0.9 × 0.9 m,
Vortex, 50 A, wheel COF 1.2. The sim takes bumper size and mass from here, and geometry and gearing from the code,
because the code is what ran on the robot.

## Shooter: `subsystems/ShooterSubsystem.java`

| Motor | Controller | ID / bus | Config | Use |
|---|---|---|---|---|
| Flywheel | SparkFlex (Vortex) | 20 / `CAN_S3` | coast, 60 A, voltage comp 12 V, `kV = 0.00205`, `p = 0.002` | `setVelocity(rpm)` in kVelocity. Hub shot `HUB_RPM = -2700` |
| Hood | SparkMax | 23 / `CAN_S3` | brake, 40 A, `p = 0.3`, output ±0.4 | `setSetpoint(rot, kPosition)`. Hub shot `HUB_POSITION = 0`, side shot `-8.5`, stow `-0.2` |
| Feeder | SparkMax | 22 / `CAN_S1` | brake, 25 A | `setVoltage(-11)` when shooting |
| Indexer (hopper) | SparkMax | 21 / `CAN_S1` | coast, 25 A | `setVoltage(-8)` when shooting |

- `toggleShooter(rpm, hood)` toggles `shooterActive`. While active, `periodic()` runs feeder −11 V and indexer −8 V
  whenever `flywheelReady()` (|velocity − desired| < 5% of |HUB_RPM| = 135 rpm), and stops them otherwise.
- Negative RPM and negative voltages are the shooting direction.
- Unknown from code (sim config, flagged as assumptions): flywheel gearing and diameter, hood angle per motor
  rotation, shooter exit point and direction on the robot, hopper capacity. The `Shoot at Hub` auto starts at
  (3.5, 4.05) facing +x toward the blue hub at about 1.1 m, which suggests the shooter faces the robot's front.

## Intake: `subsystems/GroundIntakeSubsystem.java`, `commands/Intake*.java`

| Motor | Controller | ID / bus | Config | Use |
|---|---|---|---|---|
| Roller | SparkMax | 5 / `CAN_S2` | coast, 25 A | `setThrottle(-0.9)` to intake (`IntakeRun` toggles between −0.9 and 0) |
| Drop (pivot) | SparkMax | 24 / `CAN_S2` | brake, 40 A, `p = 0.75`, output ±0.85, MAXMotion cruise 1200, accel 600, allowed error 0.15 | `kMAXMotionPositionControl` to −19 rot (down) or 0 (up) |

- `extend()` also sets the roller to +0.2 and `retract()` sets it to −0.2, so the roller state after a toggle
  depends on the order of button presses.
- Unknown from code (sim config): which side of the robot the intake is on, its width, and the pivot gearing
  (−19 motor rotations from up to down).

## Controls (`RobotContainer.configureBindings`, preset 1)

| Input | Action |
|---|---|
| Left stick | translate (field relative, inverted, 0.2 deadband) |
| Right stick X | rotate |
| X | `IntakeRun` (toggle roller −0.9 / 0) |
| A | `IntakeToggle` (deploy / stow) |
| B | toggle hub shot (−2700 rpm, hood 0) |
| Y | toggle side shot (−3000 rpm, hood −8.5) |
| View | toggle −1000 rpm, hood 0 |
| D-pad left / right | feeder +11 / −11 V |
| D-pad down / up | indexer +8 / −8 V |
| Menu | stop feeder and indexer |

## Autos

PathPlanner autos in `src/main/deploy/pathplanner/autos/`: `2-Meters`, `Shoot at Hub`, `Shoot at Left`,
`Shoot at Right`. `Shoot at Hub` runs `Backup Center` (3.5, 4.05) → (2.75, 4.05) while toggling the intake arm
and the hub shot (on for 20 s). It calls the named command `Intake Toggle Run`, but `RobotContainer` registers
`Toggle Intake Run`, so that step does nothing.

## Things that break or mislead in desktop sim

| Where | Problem | Fix ([step 11](steps/11-robot-integration.md)) |
|---|---|---|
| `Robot()` | `setUseTiming(false)` in every non-real mode. AdvantageKit then runs loops back-to-back with microsecond dt while HAL time stays the wall clock. Observed: a 56 MB log in 30 s | interactive mode keeps timing on. FAST (CI) mode adds `SimHooks` lockstep ([step 13](steps/13-headless-match-runner.md)) |
| `Robot()` | `LogFileUtil.findReplayLog()` on every sim start. It silently enters replay if AdvantageScope's temp file exists, and blocks on stdin under VS Code. Observed: it prints the prompt, then falls through under `gradlew run` | `-PsimMode=sim\|fast\|replay` via the `SIM_MODE` env var; `findReplayLog` only in replay |
| Sim GUI | defaults to Red 1, so PathPlanner flips paths. No timed match | pick Blue 1. The sim runs its own match clock |
| `DriveSubsystem` | odometry resets (PathPlanner `resetOdom`) don't move the simulated robot | pose-reset hook teleports the sim chassis |
| `SwerveModuleIONEO` | turn velocity is read from the relative encoder, which REV sim doesn't update for absolute-feedback motors | sim writes the relative encoder too |
| Turn closed loop | setpoints are in (−0.5, 0.5] rotations, the absolute encoder is [0, 1), and the 2027 REV API has no wrap input range | sim feeds the controller a position within half a turn of the setpoint |
| `DriveSubsystem` | `limelight.getRobotPose(...)` logged every loop with no camera | harmless: it's NetworkTables only and returns `new Pose3d()`. Leave it |
| `Shoot at Hub` auto | named command `Intake Toggle Run` vs registered `Toggle Intake Run` | robot bug. Leave it and tell Kurtis |
