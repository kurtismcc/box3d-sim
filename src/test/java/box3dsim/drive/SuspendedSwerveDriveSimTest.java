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

class SuspendedSwerveDriveSimTest {
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
        drive = new SwerveDriveSim(world, battery, TestConfigs.suspended(), Pose3.IDENTITY);
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

    private static double pitchDeg(Pose3 p) {
        Vec3 forward = p.transformVector(new Vec3(1, 0, 0));
        return Math.toDegrees(Math.asin(-forward.z()));
    }

    private static double rollDeg(Pose3 p) {
        Vec3 left = p.transformVector(new Vec3(0, 1, 0));
        return Math.toDegrees(Math.asin(left.z()));
    }

    @Test
    void settlesLevelWithWeightShared() {
        run(1);
        double weight = 50 * 9.81;
        double total = 0;
        for (int i = 0; i < 4; i++) {
            assertEquals(weight / 4, drive.normalLoad(i), weight / 4 * 0.02, "module " + i);
            total += drive.normalLoad(i);
        }
        assertEquals(weight, total, weight * 0.01);
        assertEquals(-weight / (4 * 70_000), drive.pose().position().z(), 2e-4);
        assertEquals(0, pitchDeg(drive.pose()), 0.1);
        assertEquals(0, rollDeg(drive.pose()), 0.1);
        assertEquals(0, drive.fieldVelocity().norm(), 1e-3);
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
        assertEquals(0, v.z(), 0.01);
        assertEquals(0, drive.yawRate(), 0.01);
    }

    @Test
    void accelerationPitchesBackAndLoadsRearWheels() {
        run(0.5);
        setAll(12, false);
        boolean rearHeavier = false;
        double minPitch = 0;
        for (int i = 0; i < Math.round(0.2 / DT); i++) {
            step();
            rearHeavier |= drive.normalLoad(2) > 1.2 * drive.normalLoad(0)
                    && drive.normalLoad(3) > 1.2 * drive.normalLoad(1);
            minPitch = Math.min(minPitch, pitchDeg(drive.pose()));
        }
        assertTrue(rearHeavier);
        // Nose up is negative pitch about +y.
        assertTrue(minPitch < -0.05, "pitch " + minPitch);
        assertTrue(minPitch > -1, "pitch " + minPitch);
    }

    @Test
    void brakeStopsShortCoastRollsOn() {
        double coast = coastDown(true);
        setUp();
        double brake = coastDown(false);
        assertTrue(coast > 2, "coast " + coast);
        assertTrue(brake < 1.0, "brake " + brake);
    }

    private double coastDown(boolean coast) {
        run(0.5);
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
    void spinInPlaceStaysPut() {
        SwerveDriveConfig cfg = drive.config();
        for (int i = 0; i < 4; i++) {
            SwerveModuleConfig mc = cfg.modules().get(i);
            drive.steer(i).setState(Math.atan2(mc.x(), -mc.y()), 0);
            drive.setModuleInputs(i, 6, false, 0, false);
        }
        run(2);
        assertEquals(0, drive.pose().position().x(), 0.05);
        assertEquals(0, drive.pose().position().y(), 0.05);
        assertTrue(drive.yawRate() > 5);
        assertEquals(0, pitchDeg(drive.pose()), 0.5);
        assertEquals(0, rollDeg(drive.pose()), 0.5);
    }

    @Test
    void wallStopsRobot() {
        BodyHandle wall = world.createBody(BodySpec.fixed(Pose3.ofXYYaw(2.1, 0, 0.5, 0)));
        world.addShape(wall, new ShapeSpec.Box(new Vec3(0.1, 2, 0.5), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.4, 0.3, 0), 0, CollisionLayers.FIELD));
        setAll(12, false);
        run(3);
        assertEquals(2 - 0.45, drive.pose().position().x(), 0.02);
        assertEquals(0, pitchDeg(drive.pose()), 2);
    }

    @Test
    void teleportPlacesLevelAtRest() {
        run(0.5);
        drive.teleport(3, 1, 1.0);
        run(0.5);
        assertEquals(3, drive.pose().position().x(), 1e-3);
        assertEquals(1, drive.pose().position().y(), 1e-3);
        assertEquals(1.0, drive.pose().rotation().yaw(), 1e-3);
        assertEquals(-50 * 9.81 / (4 * 70_000), drive.pose().position().z(), 2e-4);
    }
}
