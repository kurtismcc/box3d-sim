package box3dsim.world;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.math.Pose3;
import box3dsim.math.Quat;
import box3dsim.math.Vec3;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class Box3dWorldTest {
    private static final double DT = 0.005;
    private static final double BALL_RADIUS = 0.075;
    private static final double BALL_DENSITY = 0.215 / (4.0 / 3.0 * Math.PI * Math.pow(BALL_RADIUS, 3));

    private Box3dWorld world;

    @BeforeEach
    void setUp() {
        world = new Box3dWorld(WorldConfig.defaults());
        BodyHandle ground = world.createBody(BodySpec.fixed(Pose3.ofXYYaw(0, 0, -0.5, 0)));
        world.addShape(ground, new ShapeSpec.Box(new Vec3(10, 10, 0.5), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.8, 0.5, 0), 0, CollisionLayers.FIELD));
    }

    @AfterEach
    void tearDown() {
        world.close();
    }

    private BodyHandle ball(Vec3 position, double restitution, boolean awake) {
        BodyHandle b = world.createBody(BodySpec.dynamic(new Pose3(position, Quat.IDENTITY)).withAwake(awake));
        world.addShape(b, new ShapeSpec.Sphere(Vec3.ZERO, BALL_RADIUS),
                ShapeOptions.solid(new Material(0.6, restitution, 0), BALL_DENSITY, CollisionLayers.GAME_PIECE));
        return b;
    }

    private void run(double seconds) {
        for (int i = 0; i < Math.round(seconds / DT); i++) {
            world.step(DT, 4);
        }
    }

    @Test
    void ballRests() {
        BodyHandle b = ball(new Vec3(0, 0, 1), 0, true);
        run(3);
        assertEquals(BALL_RADIUS, world.pose(b).position().z(), 0.005);
    }

    @Test
    void ballBouncesToRestitutionSquaredHeight() {
        BodyHandle b = ball(new Vec3(0, 0, 1.0 + BALL_RADIUS), 0.5, true);
        boolean contacted = false;
        double maxAfter = 0;
        double prevVz = 0;
        for (int i = 0; i < Math.round(2.0 / DT); i++) {
            world.step(DT, 4);
            double vz = world.linearVelocity(b).z();
            if (!contacted && prevVz < 0 && vz > 0) {
                contacted = true;
            }
            if (contacted) {
                maxAfter = Math.max(maxAfter, world.pose(b).position().z() - BALL_RADIUS);
            }
            prevVz = vz;
        }
        assertTrue(contacted);
        assertEquals(0.25, maxAfter, 0.05);
    }

    @Test
    void sensorReportsBeginAndEndOnce() {
        BodyHandle sensorBody = world.createBody(BodySpec.fixed(Pose3.ofXYYaw(0, 0, 2, 0)));
        ShapeHandle sensor = world.addShape(sensorBody, new ShapeSpec.Box(new Vec3(0.5, 0.5, 0.5), Pose3.IDENTITY),
                ShapeOptions.sensor(CollisionLayers.SENSOR));
        BodyHandle b = ball(new Vec3(0, 0, 3), 0, true);
        int begins = 0;
        int ends = 0;
        boolean sawOverlap = false;
        for (int i = 0; i < Math.round(2.0 / DT); i++) {
            world.step(DT, 4);
            for (SensorEvent e : world.sensorBeginEvents()) {
                assertEquals(sensor, e.sensor());
                assertEquals(b, world.bodyOf(e.visitor()));
                begins++;
            }
            ends += world.sensorEndEvents().size();
            if (begins == 1 && ends == 0 && !world.sensorOverlaps(sensor).isEmpty()) {
                sawOverlap = true;
                assertEquals(b, world.bodyOf(world.sensorOverlaps(sensor).get(0)));
            }
        }
        assertEquals(1, begins);
        assertEquals(1, ends);
        assertTrue(sawOverlap);
    }

    @Test
    void robotBlockerStopsRobotsButNotBalls() {
        BodyHandle blocker = world.createBody(BodySpec.fixed(Pose3.ofXYYaw(0, 0, 0.5, 0)));
        world.addShape(blocker, new ShapeSpec.Box(new Vec3(2, 2, 0.05), Pose3.IDENTITY),
                ShapeOptions.solid(Material.DEFAULT, 0, CollisionLayers.ROBOT_BLOCKER));
        BodyHandle b = ball(new Vec3(0, 0, 1.0), 0, true);
        BodyHandle robot = world.createBody(BodySpec.dynamic(Pose3.ofXYYaw(1, 1, 1.0, 0)));
        world.addShape(robot, new ShapeSpec.Box(new Vec3(0.2, 0.2, 0.1), Pose3.IDENTITY),
                ShapeOptions.solid(Material.DEFAULT, 100, CollisionLayers.ROBOT));
        run(2);
        assertEquals(BALL_RADIUS, world.pose(b).position().z(), 0.01);
        assertEquals(0.55 + 0.1, world.pose(robot).position().z(), 0.02);
    }

    @Test
    void planarLocksHoldHeightAndAttitude() {
        BodyHandle box = world.createBody(BodySpec.dynamic(Pose3.ofXYYaw(0, 0, 0.3, 0))
                .withLocks(MotionLocks.PLANAR).withGravityScale(0));
        world.addShape(box, new ShapeSpec.Box(new Vec3(0.3, 0.3, 0.1), Pose3.IDENTITY),
                ShapeOptions.solid(Material.DEFAULT, 100, CollisionLayers.ROBOT));
        double startX = world.pose(box).position().x();
        for (int i = 0; i < Math.round(1.0 / DT); i++) {
            Vec3 c = world.pose(box).position();
            world.applyForce(box, new Vec3(100, 0, 0), c.plus(new Vec3(0, 0, 0.2)));
            world.step(DT, 4);
        }
        Pose3 p = world.pose(box);
        assertEquals(0.3, p.position().z(), 1e-4);
        Quat q = p.rotation();
        double tilt = 2 * Math.asin(Math.min(1, Math.hypot(q.x(), q.y())));
        assertTrue(tilt < 1e-4, "tilt " + tilt);
        assertTrue(p.position().x() > startX + 0.1);
    }

    @Test
    void bodyMovesStopAfterSleep() {
        BodyHandle b = ball(new Vec3(0, 0, 0.5), 0, true);
        boolean fellAsleep = false;
        boolean listedAfterSleep = false;
        for (int i = 0; i < Math.round(5.0 / DT); i++) {
            world.step(DT, 4);
            for (BodyMove m : world.bodyMoves()) {
                if (m.body().equals(b)) {
                    if (fellAsleep) {
                        listedAfterSleep = true;
                    }
                    if (m.fellAsleep()) {
                        fellAsleep = true;
                    }
                }
            }
        }
        assertTrue(fellAsleep);
        assertFalse(listedAfterSleep);
        assertFalse(world.isAwake(b));
    }

    @Test
    void rayHitsGround() {
        Optional<RayHit> hit = world.castRayClosest(new Vec3(0, 0, 1), new Vec3(0, 0, -2),
                new CollisionFilter(CollisionLayers.GAME_PIECE, CollisionLayers.FIELD));
        assertTrue(hit.isPresent());
        assertEquals(0.0, hit.get().point().z(), 1e-4);
        assertEquals(1.0, hit.get().normal().z(), 1e-4);
    }

    @Test
    void capsuleRestsFlat() {
        BodyHandle c = world.createBody(BodySpec.dynamic(Pose3.ofXYYaw(0, 0, 1, 0)));
        world.addShape(c, new ShapeSpec.Capsule(new Vec3(-0.2, 0, 0), new Vec3(0.2, 0, 0), 0.05),
                ShapeOptions.solid(Material.DEFAULT, 500, CollisionLayers.ROBOT));
        run(3);
        assertEquals(0.05, world.pose(c).position().z(), 0.005);
    }

    @Test
    void createdAsleepStaysQuietUntilHit() {
        BodyHandle resting = ball(new Vec3(0, 0, BALL_RADIUS), 0, false);
        assertFalse(world.isAwake(resting));
        for (int i = 0; i < 10; i++) {
            world.step(DT, 4);
            for (BodyMove m : world.bodyMoves()) {
                assertFalse(m.body().equals(resting));
            }
        }
        ball(new Vec3(0.02, 0, 0.6), 0, true);
        boolean woke = false;
        for (int i = 0; i < Math.round(1.0 / DT) && !woke; i++) {
            world.step(DT, 4);
            woke = world.isAwake(resting);
        }
        assertTrue(woke);
    }

    @Test
    void massOverrideSurvivesSensorAndSetsInertia() {
        BodyHandle box = world.createBody(BodySpec.dynamic(Pose3.ofXYYaw(0, 0, 2, 0)).withGravityScale(0));
        world.addShape(box, new ShapeSpec.Box(new Vec3(0.5, 0.5, 0.5), Pose3.IDENTITY),
                ShapeOptions.solid(Material.DEFAULT, 1, CollisionLayers.ROBOT));
        world.setMassProperties(box, 50, Vec3.ZERO, new Vec3(1, 1, 6.883));
        world.addShape(box, new ShapeSpec.Box(new Vec3(0.1, 0.1, 0.1), Pose3.ofXYYaw(0.6, 0, 0, 0)),
                ShapeOptions.sensor(CollisionLayers.SENSOR));
        assertEquals(50, world.mass(box), 1e-3);
        for (int i = 0; i < Math.round(1.0 / DT); i++) {
            world.applyTorque(box, new Vec3(0, 0, 6.883));
            world.step(DT, 4);
        }
        assertEquals(1.0, world.angularVelocity(box).z(), 0.02);
    }

    @Test
    void sensorSkipsOwnBody() {
        BodyHandle box = world.createBody(BodySpec.dynamic(Pose3.ofXYYaw(0, 0, 2, 0)).withGravityScale(0));
        world.addShape(box, new ShapeSpec.Box(new Vec3(0.2, 0.2, 0.2), Pose3.IDENTITY),
                ShapeOptions.solid(Material.DEFAULT, 1, CollisionLayers.GAME_PIECE));
        ShapeHandle s = world.addShape(box, new ShapeSpec.Box(new Vec3(0.5, 0.5, 0.5), Pose3.IDENTITY),
                ShapeOptions.sensor(CollisionLayers.SENSOR));
        run(0.1);
        List<ShapeHandle> overlaps = world.sensorOverlaps(s);
        assertTrue(overlaps.isEmpty(), overlaps.toString());
    }
}
