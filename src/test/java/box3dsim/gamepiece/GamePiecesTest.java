package box3dsim.gamepiece;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaConfig;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.world.BodyHandle;
import box3dsim.world.BodySpec;
import box3dsim.world.CollisionLayers;
import box3dsim.world.Material;
import box3dsim.world.ShapeOptions;
import box3dsim.world.ShapeSpec;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class GamePiecesTest {
    private static final double G = 9.81;
    private static final GamePieceType BALL =
            new GamePieceType("Ball", 0.075, 0.215, new Material(0.6, 0.45, 0.05), 0.5, 0.5, 0.35, 0.3, 0, 0);

    private Arena arena;
    private GamePieces pieces;

    @BeforeEach
    void setUp() {
        arena = new Arena(ArenaConfig.defaults());
        BodyHandle ground = arena.world().createBody(BodySpec.fixed(Pose3.ofXYYaw(0, 0, -0.5, 0)));
        arena.world().addShape(ground, new ShapeSpec.Box(new Vec3(50, 50, 0.5), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.8, 0, 0), 0, CollisionLayers.FIELD));
        pieces = arena.gamePieces();
    }

    @AfterEach
    void tearDown() {
        arena.close();
    }

    /** x where the piece's center first descends through z = radius, interpolated between steps. */
    private double landingX(GamePiece p) {
        double prevX = p.pose().position().x();
        double prevZ = p.pose().position().z();
        for (int i = 0; i < 2000; i++) {
            arena.step();
            Vec3 pos = p.pose().position();
            if (pos.z() <= BALL.radius() && prevZ > BALL.radius()) {
                double f = (prevZ - BALL.radius()) / (prevZ - pos.z());
                return prevX + f * (pos.x() - prevX);
            }
            prevX = pos.x();
            prevZ = pos.z();
        }
        throw new AssertionError("never landed");
    }

    private static Vec3 launch(double speed, double angle) {
        return new Vec3(speed * Math.cos(angle), 0, speed * Math.sin(angle));
    }

    @Test
    void vacuumParabolaMatchesAnalyticRange() {
        pieces.setAerodynamicsEnabled(false);
        double v = 8;
        double th = Math.toRadians(45);
        GamePiece p = pieces.spawn(BALL, new Vec3(0, 0, 1), launch(v, th), Vec3.ZERO, true);
        double expected = v * Math.cos(th) / G
                * (v * Math.sin(th) + Math.sqrt(Math.pow(v * Math.sin(th), 2) + 2 * G * (1 - BALL.radius())));
        assertEquals(expected, landingX(p), 0.02);
    }

    @Test
    void dragShortensAndBackspinLengthens() {
        double v = 8;
        double th = Math.toRadians(45);
        pieces.setAerodynamicsEnabled(false);
        double vacuum = landingX(pieces.spawn(BALL, new Vec3(0, 0, 1), launch(v, th), Vec3.ZERO, true));
        pieces.setAerodynamicsEnabled(true);
        double drag = landingX(pieces.spawn(BALL, new Vec3(0, 5, 1), launch(v, th), Vec3.ZERO, true));
        Vec3 backspin = new Vec3(0, -v / BALL.radius(), 0);
        double spin = landingX(pieces.spawn(BALL, new Vec3(0, 10, 1), launch(v, th), backspin, true));
        System.out.printf("range vacuum %.3f, drag %.3f, drag+backspin %.3f%n", vacuum, drag, spin);
        double shortening = 1 - drag / vacuum;
        assertTrue(shortening > 0.05 && shortening < 0.20, "drag shortening " + shortening);
        assertTrue(spin > drag, "backspin " + spin + " vs drag " + drag);
    }

    @Test
    void restingPieceSleepsAndKeepsPose() {
        GamePiece p = pieces.spawn(BALL, new Vec3(0, 0, 0.5), Vec3.ZERO, Vec3.ZERO, true);
        arena.advance(3);
        assertFalse(p.isAwake());
        assertEquals(0, pieces.awakeCount());
        List<Pose3> poses = pieces.poses(BALL);
        assertEquals(1, poses.size());
        assertEquals(BALL.radius(), poses.get(0).position().z(), 0.005);
    }

    @Test
    void outOfFieldPiecesAreRemoved() {
        pieces.setFieldBounds(0, 10, 0, 10);
        GamePiece p = pieces.spawn(BALL, new Vec3(9, 5, 1), new Vec3(8, 0, 2), Vec3.ZERO, true);
        arena.advance(2);
        assertFalse(p.isAlive());
        assertEquals(1, pieces.outOfFieldCount());
        assertEquals(0, pieces.count(BALL));
    }

    @Test
    void idsAreUniqueAndRemovedShapesUnmapped() {
        GamePiece a = pieces.spawn(BALL, new Vec3(0, 0, 0.075), Vec3.ZERO, Vec3.ZERO, false);
        var shape = a.shape();
        pieces.remove(a);
        GamePiece b = pieces.spawn(BALL, new Vec3(0, 0, 0.075), Vec3.ZERO, Vec3.ZERO, false);
        assertNotEquals(a.id(), b.id());
        assertTrue(pieces.byShape(shape).isEmpty() || pieces.byShape(shape).get() == b);
        assertTrue(pieces.byShape(b.shape()).isPresent());
    }

    @Test
    void pileStaysStableWhenHit() {
        List<GamePiece> pile = new ArrayList<>();
        for (int i = 0; i < 20; i++) {
            for (int j = 0; j < 20; j++) {
                pile.add(pieces.spawn(BALL, new Vec3(1 + i * 0.16, j * 0.16, 0.075), Vec3.ZERO, Vec3.ZERO, false));
            }
        }
        GamePiece striker = pieces.spawn(BALL, new Vec3(0, 1.5, 0.075), new Vec3(10, 0, 0), Vec3.ZERO, true);
        pile.add(striker);
        long total = 0;
        int steps = (int) Math.round(2.0 / arena.config().physicsDt());
        for (int s = 0; s < steps; s++) {
            arena.step();
            total += arena.lastStepStats().physicsNanos();
        }
        for (GamePiece p : pile) {
            double z = p.pose().position().z();
            assertTrue(z >= 0.07 && z <= 0.6, "z " + z);
        }
        System.out.printf("401-ball pile: mean physics step %.3f ms, awake at end %d%n",
                total / 1e6 / steps, pieces.awakeCount());
    }
}
