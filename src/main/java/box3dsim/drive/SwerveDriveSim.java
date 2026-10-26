package box3dsim.drive;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaParticipant;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.motor.Battery;
import box3dsim.motor.DcMotor;
import box3dsim.motor.ElectricalLoad;
import box3dsim.motor.LumpedMechanism;
import box3dsim.world.BodyHandle;
import box3dsim.world.BodySpec;
import box3dsim.world.CollisionLayers;
import box3dsim.world.Material;
import box3dsim.world.MotionLocks;
import box3dsim.world.ShapeOptions;
import box3dsim.world.ShapeSpec;
import box3dsim.world.SimWorld;
import java.util.List;

/**
 * Swerve chassis as a planar body that never touches the carpet. All traction comes from a per-wheel slip-velocity
 * tire model; normal loads come from static weight plus analytic load transfer.
 */
public final class SwerveDriveSim implements ArenaParticipant, ElectricalLoad {
    private static final double GRAVITY = 9.81;
    private static final int WHEEL_MICRO_STEPS = 10;
    private static final double ACCEL_FILTER_TIME_CONSTANT = 0.05;
    private static final Material BUMPER = new Material(0.5, 0.1, 0);
    private static final Material FRAME = new Material(0.3, 0.2, 0);

    private final SimWorld world;
    private final Battery battery;
    private final SwerveDriveConfig config;
    private final BodyHandle body;
    private final Module[] modules;
    private final GyroSim gyro;
    private final double meanX2;
    private final double meanY2;

    private Pose3 pose;
    private Vec3 velocity = Vec3.ZERO;
    private double yawRate;
    private Vec3 prevVelocity = Vec3.ZERO;
    private double accelX;
    private double accelY;

    private static final class Module {
        final SwerveModuleConfig cfg;
        final LumpedMechanism steer;
        double driveVolts;
        boolean driveCoast;
        double steerVolts;
        boolean steerCoast;
        double wheelPosition;
        double wheelVelocity;
        double driveApplied;
        double driveStator;
        double driveSupply;
        double normalLoad;

        Module(SwerveModuleConfig cfg, Battery battery) {
            this.cfg = cfg;
            this.steer = new LumpedMechanism(
                    LumpedMechanism.Config.of(cfg.steerMotor(), cfg.steerGearing(), cfg.steerMoi())
                            .withFriction(cfg.steerFrictionTorque(), 0),
                    battery);
        }
    }

    public SwerveDriveSim(SimWorld world, Battery battery, SwerveDriveConfig config, Pose3 startPose) {
        this.world = world;
        this.battery = battery;
        this.config = config;
        double yaw = startPose.rotation().yaw();
        Pose3 start = Pose3.ofXYYaw(startPose.position().x(), startPose.position().y(), 0, yaw);
        this.body = world.createBody(BodySpec.dynamic(start)
                .withLocks(MotionLocks.PLANAR)
                .withGravityScale(0)
                .withBullet(true)
                .withSleep(false)
                .withName("swerve chassis"));

        double hx = config.bumperLengthX() / 2;
        double hy = config.bumperWidthY() / 2;
        double bottom = config.bumperBottom();
        double top = bottom + config.bumperHeight();
        ShapeOptions bumper = ShapeOptions.solid(BUMPER, 1.0, CollisionLayers.ROBOT);
        world.addShape(body, new ShapeSpec.Box(new Vec3(hx, hy, config.bumperHeight() / 2),
                Pose3.ofXYYaw(0, 0, bottom + config.bumperHeight() / 2, 0)), bumper);
        // Skirt under the bumper so 15 cm balls can't slip beneath the robot.
        world.addShape(body, new ShapeSpec.Box(new Vec3(hx - 0.01, hy - 0.01, bottom / 2 - 0.005),
                Pose3.ofXYYaw(0, 0, bottom / 2 + 0.005, 0)), bumper);
        double inset = config.frameInset();
        double frameHalfHeight = (config.robotHeight() - top) / 2;
        world.addShape(body, new ShapeSpec.Box(new Vec3(hx - inset, hy - inset, frameHalfHeight),
                Pose3.ofXYYaw(0, 0, top + frameHalfHeight, 0)), ShapeOptions.solid(FRAME, 1.0, CollisionLayers.ROBOT));
        applyMass();

        List<SwerveModuleConfig> cfgs = config.modules();
        this.modules = new Module[cfgs.size()];
        double sx = 0;
        double sy = 0;
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(cfgs.get(i), battery);
            sx += cfgs.get(i).x() * cfgs.get(i).x();
            sy += cfgs.get(i).y() * cfgs.get(i).y();
        }
        this.meanX2 = sx / modules.length;
        this.meanY2 = sy / modules.length;
        this.gyro = new GyroSim(config.gyro(), yaw);
        this.pose = start;
    }

    private void applyMass() {
        world.setMassProperties(body, config.mass(), new Vec3(0, 0, config.cgHeight()),
                new Vec3(config.yawMoi(), config.yawMoi(), config.yawMoi()));
    }

    /** Re-applies the configured mass if something (a shape added later) reset it. */
    void ensureMass() {
        if (Math.abs(world.mass(body) - config.mass()) > 1e-6) {
            applyMass();
        }
    }

    public int moduleCount() {
        return modules.length;
    }

    /** Positive drive volts roll the wheel toward the module heading. */
    public void setModuleInputs(int i, double driveVolts, boolean driveCoast, double steerVolts, boolean steerCoast) {
        Module m = modules[i];
        m.driveVolts = driveVolts;
        m.driveCoast = driveCoast;
        m.steerVolts = steerVolts;
        m.steerCoast = steerCoast;
    }

    /** Computes tire forces and applies them to the chassis for the next world step. */
    public void prePhysics(double dt) {
        ensureMass();
        pose = world.pose(body);
        velocity = world.linearVelocity(body);
        yawRate = world.angularVelocity(body).z();
        double yaw = pose.rotation().yaw();
        Vec3 c = pose.position();
        int n = modules.length;
        double weight = config.mass() * GRAVITY / n;
        double vBat = battery.voltage();
        TireConfig tire = config.tire();

        for (Module m : modules) {
            SwerveModuleConfig mc = m.cfg;
            if (config.loadTransfer()) {
                double transfer = config.mass() * config.cgHeight()
                        * (accelX * mc.x() / (n * meanX2) + accelY * mc.y() / (n * meanY2));
                m.normalLoad = Math.clamp(weight - transfer, 0.05 * weight, 2 * weight);
            } else {
                m.normalLoad = weight;
            }

            m.steer.setInput(m.steerVolts, m.steerCoast);
            m.steer.step(dt);

            Vec3 p = pose.transformPoint(new Vec3(mc.x(), mc.y(), 0));
            double rx = p.x() - c.x();
            double ry = p.y() - c.y();
            double vx = velocity.x() - yawRate * ry;
            double vy = velocity.y() + yawRate * rx;
            double heading = yaw + m.steer.position();
            double hx = Math.cos(heading);
            double hy = Math.sin(heading);
            double u = vx * hx + vy * hy;
            double w = -vx * hy + vy * hx;

            DcMotor motor = mc.driveMotor();
            double g = mc.driveGearing();
            double r = mc.wheelRadius();
            double j = mc.wheelMoi();
            double a = g * motor.kt() / motor.resistance();
            double b = g * g * motor.kt() / (motor.kv() * motor.resistance());
            double volts = Math.clamp(m.driveVolts, -vBat, vBat);
            boolean open = m.driveCoast && Math.abs(volts) < 1e-9;
            double delta = dt / WHEEL_MICRO_STEPS;
            double frictionStep = delta * mc.driveFrictionTorque() / j;

            double sumFh = 0;
            double sumFl = 0;
            double sumStator = 0;
            for (int k = 0; k < WHEEL_MICRO_STEPS; k++) {
                double sLong = u - m.wheelVelocity * r;
                double sigma = Math.hypot(sLong, w);
                double fh = 0;
                double fl = 0;
                if (sigma > 1e-9) {
                    double mu = tire.kineticMu()
                            + (tire.staticMu() - tire.kineticMu()) * Math.exp(-sigma / tire.slipDecayVelocity());
                    double mag = mu * m.normalLoad * Math.tanh(sigma / tire.slipVelocity());
                    fh = -mag * sLong / sigma;
                    fl = -mag * w / sigma;
                }
                double motorTorque = open ? 0 : a * volts - b * m.wheelVelocity;
                double wNext = m.wheelVelocity + delta / j * (motorTorque - r * fh);
                wNext = Math.abs(wNext) <= frictionStep ? 0 : wNext - Math.copySign(frictionStep, wNext);
                m.wheelVelocity = wNext;
                m.wheelPosition += wNext * delta;
                sumFh += fh;
                sumFl += fl;
                sumStator += open ? 0 : (volts - g * wNext / motor.kv()) / motor.resistance();
            }
            double fh = sumFh / WHEEL_MICRO_STEPS;
            double fl = sumFl / WHEEL_MICRO_STEPS;
            world.applyForce(body, new Vec3(fh * hx - fl * hy, fh * hy + fl * hx, 0), p);

            m.driveApplied = open ? 0 : volts;
            m.driveStator = sumStator / WHEEL_MICRO_STEPS;
            m.driveSupply = ElectricalLoad.supplyFromStator(m.driveStator, m.driveApplied, vBat);
        }
    }

    @Override
    public void prePhysics(Arena arena, double dt) {
        prePhysics(dt);
    }

    @Override
    public void postPhysics(Arena arena, double dt) {
        postPhysics(dt);
    }

    /** Reads back the chassis state after the world step. */
    public void postPhysics(double dt) {
        pose = world.pose(body);
        velocity = world.linearVelocity(body);
        yawRate = world.angularVelocity(body).z();
        double yaw = pose.rotation().yaw();

        double ax = (velocity.x() - prevVelocity.x()) / dt;
        double ay = (velocity.y() - prevVelocity.y()) / dt;
        prevVelocity = velocity;
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);
        double axRobot = ax * cos + ay * sin;
        double ayRobot = -ax * sin + ay * cos;
        double alpha = dt / (ACCEL_FILTER_TIME_CONSTANT + dt);
        accelX += alpha * (axRobot - accelX);
        accelY += alpha * (ayRobot - accelY);

        gyro.update(yaw, dt);
    }

    public double wheelPositionRad(int i) {
        return modules[i].wheelPosition;
    }

    public double wheelVelocityRadPerSec(int i) {
        return modules[i].wheelVelocity;
    }

    /** Module azimuth, rad, continuous, CCW positive, 0 = rolling along robot +x. */
    public double steerPositionRad(int i) {
        return modules[i].steer.position();
    }

    public double steerVelocityRadPerSec(int i) {
        return modules[i].steer.velocity();
    }

    public double driveAppliedVolts(int i) {
        return modules[i].driveApplied;
    }

    public double driveStatorCurrent(int i) {
        return modules[i].driveStator;
    }

    public double steerAppliedVolts(int i) {
        return modules[i].steer.appliedVolts();
    }

    public double steerStatorCurrent(int i) {
        return modules[i].steer.statorCurrent();
    }

    /** True pose, z = 0. */
    public Pose3 pose() {
        return pose;
    }

    public Vec3 fieldVelocity() {
        return velocity;
    }

    public double yawRate() {
        return yawRate;
    }

    public GyroSim gyro() {
        return gyro;
    }

    /** Zeroes chassis velocity and wheel spin; the gyro reading is unchanged. */
    public void teleport(double x, double y, double yaw) {
        Pose3 p = Pose3.ofXYYaw(x, y, 0, yaw);
        world.setPose(body, p);
        world.setVelocity(body, Vec3.ZERO, Vec3.ZERO);
        pose = p;
        velocity = Vec3.ZERO;
        prevVelocity = Vec3.ZERO;
        yawRate = 0;
        accelX = 0;
        accelY = 0;
        for (Module m : modules) {
            m.wheelVelocity = 0;
        }
        gyro.rebase(yaw);
    }

    public BodyHandle body() {
        return body;
    }

    public SwerveDriveConfig config() {
        return config;
    }

    @Override
    public double supplyCurrent() {
        double total = 0;
        for (Module m : modules) {
            total += m.driveSupply + m.steer.supplyCurrent();
        }
        return total;
    }

    LumpedMechanism steer(int i) {
        return modules[i].steer;
    }

    double normalLoad(int i) {
        return modules[i].normalLoad;
    }

    void setWheelVelocity(int i, double radPerSec) {
        modules[i].wheelVelocity = radPerSec;
    }
}
