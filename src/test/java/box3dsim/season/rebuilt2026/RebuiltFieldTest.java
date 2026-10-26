package box3dsim.season.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.arena.Alliance;
import box3dsim.arena.Arena;
import box3dsim.arena.ArenaConfig;
import box3dsim.gamepiece.GamePiece;
import box3dsim.gamepiece.GamePieces;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.world.BodyHandle;
import box3dsim.world.BodySpec;
import box3dsim.world.CollisionLayers;
import box3dsim.world.Material;
import box3dsim.world.MotionLocks;
import box3dsim.world.SensorEvent;
import box3dsim.world.ShapeHandle;
import box3dsim.world.ShapeOptions;
import box3dsim.world.ShapeSpec;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RebuiltFieldTest {
    private Arena arena;
    private RebuiltFieldHandles field;
    private GamePieces pieces;

    @BeforeEach
    void setUp() {
        arena = new Arena(ArenaConfig.defaults());
        field = RebuiltField.build(arena.world());
        pieces = arena.gamePieces();
    }

    @AfterEach
    void tearDown() {
        arena.close();
    }

    private GamePiece fuel(Vec3 position, Vec3 velocity) {
        return pieces.spawn(RebuiltFuel.TYPE, position, velocity, Vec3.ZERO, true);
    }

    /** A FUEL already rolling without slip along the floor. */
    private GamePiece rollingFuel(Vec3 position, Vec3 velocity) {
        Vec3 spin = new Vec3(0, 0, 1).cross(velocity).times(1 / RebuiltFuel.TYPE.radius());
        return pieces.spawn(RebuiltFuel.TYPE, position, velocity, spin, true);
    }

    @Test
    void layoutCountsAndSpacing() {
        assertEquals(360, RebuiltFuelLayout.neutralZone().size());
        assertEquals(24, RebuiltFuelLayout.depot(Alliance.BLUE).size());
        assertEquals(24, RebuiltFuelLayout.chuteDisplay(Alliance.RED).size());
        List<Vec3> all = new ArrayList<>(RebuiltFuelLayout.neutralZone());
        all.addAll(RebuiltFuelLayout.depot(Alliance.BLUE));
        all.addAll(RebuiltFuelLayout.depot(Alliance.RED));
        for (int i = 0; i < all.size(); i++) {
            Vec3 p = all.get(i);
            assertTrue(p.x() > 0 && p.x() < RebuiltField.LENGTH && p.y() > 0 && p.y() < RebuiltField.WIDTH);
            for (int j = i + 1; j < all.size(); j++) {
                assertTrue(p.minus(all.get(j)).norm() >= 0.15, "too close: " + p + " " + all.get(j));
            }
        }
    }

    @Test
    void mirrorIsAnInvolution() {
        Vec3 p = new Vec3(1.2, 3.4, 0.5);
        Vec3 back = RebuiltField.mirror(RebuiltField.mirror(p));
        assertEquals(p.x(), back.x(), 1e-12);
        assertEquals(p.y(), back.y(), 1e-12);
        assertEquals(RebuiltField.LENGTH - 4.6255, RebuiltField.hubCenter(Alliance.RED).x(), 1e-12);
    }

    @Test
    void hubCatchesFuel() {
        GamePiece f = fuel(new Vec3(4.6255, 4.0346, 2.5), Vec3.ZERO);
        ShapeHandle sensor = field.hubSensors().get(Alliance.BLUE);
        boolean sensed = false;
        for (int i = 0; i < 250 && !sensed; i++) {
            arena.step();
            for (SensorEvent e : arena.world().sensorBeginEvents()) {
                sensed |= e.sensor().equals(sensor) && pieces.byShape(e.visitor()).orElse(null) == f;
            }
        }
        assertTrue(sensed);
        arena.advance(2);
        assertEquals(1.475, f.pose().position().z(), 0.01);
    }

    @Test
    void hubRimDeflects() {
        fuel(new Vec3(4.0285 + 0.032, 4.0346, 2.5), Vec3.ZERO);
        arena.advance(2);
        assertTrue(arena.world().sensorOverlaps(field.hubSensors().get(Alliance.BLUE)).isEmpty());
    }

    @Test
    void fuelRollsUnderTrench() {
        GamePiece f = rollingFuel(new Vec3(3.0, 0.6, 0.075), new Vec3(2, 0, 0));
        arena.advance(2);
        assertTrue(f.pose().position().x() > 5.3, "x " + f.pose().position().x());
    }

    @Test
    void fuelRidesOverBumpButRobotPassesThrough() {
        GamePiece f = fuel(new Vec3(3.5, 2.5, 0.075), new Vec3(3, 0, 0));
        BodyHandle robot = arena.world().createBody(BodySpec.dynamic(Pose3.ofXYYaw(3.5, 5.5, 0.2, 0))
                .withLocks(MotionLocks.PLANAR).withGravityScale(0).withSleep(false));
        arena.world().addShape(robot, new ShapeSpec.Box(new Vec3(0.45, 0.45, 0.15), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.5, 0.1, 0), 100, CollisionLayers.ROBOT));
        arena.world().setVelocity(robot, new Vec3(2, 0, 0), Vec3.ZERO);
        double maxZ = 0;
        for (int i = 0; i < Math.round(1.5 / arena.config().physicsDt()); i++) {
            arena.step();
            maxZ = Math.max(maxZ, f.pose().position().z());
        }
        assertTrue(maxZ > 0.2, "max z " + maxZ);
        assertTrue(arena.world().pose(robot).position().x() > 5.3, "robot x " + arena.world().pose(robot).position().x());
    }

    @Test
    void bumperPushesFuelIntoCorral() {
        fuel(new Vec3(0.55, 0.45, 0.075), Vec3.ZERO);
        BodyHandle robot = arena.world().createBody(BodySpec.dynamic(Pose3.ofXYYaw(0.9, 0.45, 0.105, 0))
                .withLocks(MotionLocks.PLANAR).withGravityScale(0).withSleep(false));
        arena.world().addShape(robot, new ShapeSpec.Box(new Vec3(0.25, 0.25, 0.041), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.5, 0.1, 0), 400, CollisionLayers.ROBOT));
        ShapeHandle sensor = field.corralSensors().get(Alliance.BLUE);
        boolean sensed = false;
        for (int i = 0; i < 750 && !sensed; i++) {
            arena.world().setVelocity(robot, new Vec3(-1, 0, 0), Vec3.ZERO);
            arena.step();
            for (SensorEvent e : arena.world().sensorBeginEvents()) {
                sensed |= e.sensor().equals(sensor);
            }
        }
        assertTrue(sensed);
    }

    @Test
    void guardrailBouncesFuelBack() {
        GamePiece f = fuel(new Vec3(8.0, 1.0, 0.075), new Vec3(0, -4, 0));
        double minY = Double.MAX_VALUE;
        for (int i = 0; i < 500; i++) {
            arena.step();
            minY = Math.min(minY, f.pose().position().y());
        }
        assertTrue(minY >= 0.075 - 0.01, "min y " + minY);
        assertTrue(f.pose().position().y() > 0.2);
    }

    @Test
    void startingFuelStaysSettled() {
        List<GamePiece> spawned = new ArrayList<>();
        List<Vec3> positions = new ArrayList<>(RebuiltFuelLayout.neutralZone());
        positions.addAll(RebuiltFuelLayout.depot(Alliance.BLUE));
        positions.addAll(RebuiltFuelLayout.depot(Alliance.RED));
        for (Vec3 p : positions) {
            spawned.add(pieces.spawn(RebuiltFuel.TYPE, p, Vec3.ZERO, Vec3.ZERO, false));
        }
        arena.advance(1);
        assertEquals(0, pieces.awakeCount());
        for (int i = 0; i < spawned.size(); i++) {
            assertTrue(spawned.get(i).pose().position().minus(positions.get(i)).norm() < 1e-3);
        }
    }
}
