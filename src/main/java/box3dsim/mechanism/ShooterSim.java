package box3dsim.mechanism;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaParticipant;
import box3dsim.drive.SwerveDriveSim;
import box3dsim.gamepiece.GamePieceType;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.motor.Battery;
import box3dsim.motor.ElectricalLoad;
import box3dsim.motor.LumpedMechanism;
import java.util.Optional;

/** Flywheel and hood shooter that launches real game piece bodies and loses flywheel energy per shot. */
public final class ShooterSim implements ArenaParticipant, ElectricalLoad, Launcher {
    private final Arena arena;
    private final SwerveDriveSim drive;
    private final ShooterConfig config;
    private final LaunchListener listener;
    private final LumpedMechanism flywheel;
    private final LumpedMechanism hood;
    private int shots;
    private Vec3 lastExitVelocity;

    public ShooterSim(Arena arena, SwerveDriveSim drive, Battery battery, ShooterConfig config,
            LaunchListener listener) {
        this.arena = arena;
        this.drive = drive;
        this.config = config;
        this.listener = listener;
        this.flywheel = new LumpedMechanism(config.flywheel(), battery);
        this.hood = new LumpedMechanism(config.hood(), battery);
    }

    public void setFlywheelInput(double volts, boolean coastAtZero) {
        flywheel.setInput(volts, coastAtZero);
    }

    public void setHoodInput(double volts, boolean coastAtZero) {
        hood.setInput(volts, coastAtZero);
    }

    public LumpedMechanism flywheel() {
        return flywheel;
    }

    public LumpedMechanism hood() {
        return hood;
    }

    /** Current launch pitch above horizontal, rad. */
    public double launchPitch() {
        return config.hoodPositionToPitch().applyAsDouble(hood.position());
    }

    public int shotsFired() {
        return shots;
    }

    /** Field-frame velocity of the last launched piece. */
    public Optional<Vec3> lastExitVelocity() {
        return Optional.ofNullable(lastExitVelocity);
    }

    @Override
    public void prePhysics(Arena arena, double dt) {
        flywheel.step(dt);
        hood.step(dt);
    }

    @Override
    public boolean launch(GamePieceType type) {
        double surface = config.shotDirectionSign() * flywheel.velocity() * config.flywheelRadius();
        double exitSpeed = config.exitSpeedRatio() * surface;
        if (exitSpeed < config.minExitSpeed()) {
            return false;
        }
        double pitch = launchPitch();
        Pose3 robot = drive.pose();
        double yaw0 = config.exitYawRobot();
        Vec3 dir = robot.transformVector(new Vec3(Math.cos(pitch) * Math.cos(yaw0), Math.cos(pitch) * Math.sin(yaw0),
                Math.sin(pitch)));
        Vec3 p = robot.transformPoint(config.exitPointRobot());
        Vec3 r = p.minus(robot.position());
        Vec3 chassis = drive.fieldVelocity();
        double omega = drive.yawRate();
        Vec3 v = new Vec3(chassis.x() - omega * r.y(), chassis.y() + omega * r.x(), 0).plus(dir.times(exitSpeed));
        Vec3 spinAxis = dir.cross(new Vec3(0, 0, 1)).normalized();
        Vec3 spin = spinAxis.times(config.spinRatio() * exitSpeed / type.radius());
        arena.gamePieces().spawn(type, p, v, spin, true);

        double ballInertia = 0.4 * type.mass() * type.radius() * type.radius();
        double energy = 0.5 * type.mass() * exitSpeed * exitSpeed + 0.5 * ballInertia * spin.normSquared();
        flywheel.removeEnergy(energy / config.energyEfficiency());

        listener.onLaunch(robot, v);
        shots++;
        lastExitVelocity = v;
        return true;
    }

    @Override
    public void reset(Arena arena) {
        shots = 0;
        lastExitVelocity = null;
    }

    @Override
    public double supplyCurrent() {
        return flywheel.supplyCurrent() + hood.supplyCurrent();
    }
}
