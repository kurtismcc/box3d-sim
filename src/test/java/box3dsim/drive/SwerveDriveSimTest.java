package box3dsim.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.motor.Battery;
import box3dsim.motor.DcMotor;
import box3dsim.world.BodyHandle;
import box3dsim.world.BodySpec;
import box3dsim.world.Box3dWorld;
import box3dsim.world.CollisionLayers;
import box3dsim.world.Material;
import box3dsim.world.ShapeOptions;
import box3dsim.world.ShapeSpec;
import box3dsim.world.WorldConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SwerveDriveSimTest {
    private static final double DT = 0.004;

    private Box3dWorld world;
    private Battery battery;
    private SwerveDriveSim drive;

    @BeforeEach
    void setUp() {
        world = new Box3dWorld(WorldConfig.defaults());
        BodyHandle ground = world.createBody(BodySpec.fixed(Pose3.ofXYYaw(0, 0, -0.5, 0)));
        world.addShape(ground, new ShapeSpec.Box(new Vec3(50, 50, 0.5), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.8, 0, 0), 0, CollisionLayers.FIELD));
        battery = Battery.defaults();
        drive = new SwerveDriveSim(world, battery, TestConfigs.maxSwerve(), Pose3.IDENTITY);
    }

    @AfterEach
    void tearDown() {
        world.close();
    }

    private void step() {
        drive.prePhysics(DT);
        world.step(DT, 4);
        drive.postPhysics(DT);
        battery.update(drive.supplyCurrent(), DT);
    }

    private void setAll(double driveVolts, boolean driveCoast) {
        for (int i = 0; i < drive.moduleCount(); i++) {
            drive.setModuleInputs(i, driveVolts, driveCoast, 0, false);
        }
    }

    private void run(double seconds) {
        for (int i = 0; i < Math.round(seconds / DT); i++) {
            step();
        }
    }

    @Test
    void straightLineReachesModelTopSpeed() {
        setAll(12, false);
        run(3);
        DcMotor m = DcMotor.neoVortex(1);
        double frictionCurrent = 0.15 / (3.625 * m.kt());
        double expected = (battery.voltage() - frictionCurrent * m.resistance()) * m.kv() * 0.0381 / 3.625;
        Vec3 v = drive.fieldVelocity();
        assertEquals(expected, v.x(), expected * 0.05);
        assertEquals(0, v.y(), 0.01);
        assertEquals(0, drive.yawRate(), 0.01);
    }

    @Test
    void drivingSlipPushesForward() {
        setAll(12, false);
        run(0.1);
        assertTrue(drive.fieldVelocity().x() > 0);
        for (int i = 0; i < 4; i++) {
            assertTrue(drive.wheelVelocityRadPerSec(i) * 0.0381 > drive.fieldVelocity().x());
        }
    }

    @Test
    void accelerationIsTractionLimited() {
        setAll(12, false);
        double peak = 0;
        double prev = 0;
        for (int i = 0; i < Math.round(0.2 / DT); i++) {
            step();
            double vx = drive.fieldVelocity().x();
            peak = Math.max(peak, (vx - prev) / DT);
            prev = vx;
        }
        assertTrue(peak <= 1.2 * 9.81 * 1.05, "peak accel " + peak);
    }

    @Test
    void brakeStopsShortCoastRollsOn() {
        double coast = coastDown(true);
        setUp();
        double brake = coastDown(false);
        assertTrue(coast > 2, "coast " + coast);
        assertTrue(brake < 1.0, "brake " + brake);
        assertTrue(brake < coast / 3);
    }

    private double coastDown(boolean coast) {
        world.setVelocity(drive.body(), new Vec3(3, 0, 0), Vec3.ZERO);
        for (int i = 0; i < 4; i++) {
            drive.setWheelVelocity(i, 3 / 0.0381);
        }
        setAll(0, coast);
        run(4);
        double distance = drive.pose().position().x();
        world.close();
        return distance;
    }

    @Test
    void spinInPlace() {
        SwerveDriveConfig cfg = drive.config();
        for (int i = 0; i < 4; i++) {
            SwerveModuleConfig mc = cfg.modules().get(i);
            drive.steer(i).setState(Math.atan2(mc.x(), -mc.y()), 0);
            drive.setModuleInputs(i, 6, false, 0, false);
        }
        double unwrapped = 0;
        double prevYaw = drive.pose().rotation().yaw();
        for (int i = 0; i < Math.round(2.0 / DT); i++) {
            step();
            double yaw = drive.pose().rotation().yaw();
            unwrapped += GyroSim.wrapToPi(yaw - prevYaw);
            prevYaw = yaw;
        }
        assertEquals(0, drive.fieldVelocity().x(), 0.05);
        assertEquals(0, drive.fieldVelocity().y(), 0.05);
        assertTrue(drive.yawRate() > 0);
        assertEquals(unwrapped, drive.gyro().yawRad(), 1e-6);
    }

    @Test
    void lateralTireForceStopsSideslip() {
        world.setVelocity(drive.body(), new Vec3(0, 2, 0), Vec3.ZERO);
        setAll(0, false);
        run(0.3);
        assertTrue(Math.abs(drive.fieldVelocity().y()) < 0.05, "vy " + drive.fieldVelocity().y());
    }

    @Test
    void accelerationShiftsLoadRearward() {
        setAll(12, false);
        boolean rearHeavier = false;
        for (int i = 0; i < Math.round(0.2 / DT); i++) {
            step();
            if (drive.normalLoad(2) > drive.normalLoad(0) && drive.normalLoad(3) > drive.normalLoad(1)) {
                rearHeavier = true;
            }
        }
        assertTrue(rearHeavier);
    }

    @Test
    void wallStopsRobotWithoutTunneling() {
        BodyHandle wall = world.createBody(BodySpec.fixed(Pose3.ofXYYaw(2.1, 0, 0.5, 0)));
        world.addShape(wall, new ShapeSpec.Box(new Vec3(0.1, 2, 0.5), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.4, 0.3, 0), 0, CollisionLayers.FIELD));
        setAll(12, false);
        run(3);
        assertEquals(2 - 0.45, drive.pose().position().x(), 0.02);
        assertEquals(0, drive.pose().position().z(), 1e-6);
    }

    @Test
    void thirtySecondsOfDrivingIsCheap() {
        setAll(6, false);
        long t0 = System.nanoTime();
        run(30);
        double seconds = (System.nanoTime() - t0) / 1e9;
        System.out.printf("30 s of swerve sim took %.3f s%n", seconds);
        assertTrue(seconds < 5, "too slow: " + seconds);
    }
}
