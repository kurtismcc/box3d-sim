package box3dsim.season.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaConfig;
import box3dsim.drive.SwerveDriveConfig;
import box3dsim.drive.SwerveDriveSim;
import box3dsim.drive.TestConfigs;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class BumpCrossingTest {
    /** Blue BUMP between the HUB and the right guardrail spans x 4.06–5.19, y 1.58–3.44. */
    private static final double BUMP_LANE_Y = 2.5;
    /** Under the blue TRENCH arm, flat. */
    private static final double TRENCH_LANE_Y = 0.64;
    private static final double START_X = 3.2;
    private static final double END_X = 6.0;

    private Arena arena;
    private SwerveDriveSim drive;

    private record Run(double seconds, double maxNoseUpDeg, double maxNoseDownDeg, double wheelDistance,
            double groundDistance) {}

    @AfterEach
    void tearDown() {
        arena.close();
    }

    private Run drive(SwerveDriveConfig cfg, double y, double volts, double timeout) {
        arena = new Arena(ArenaConfig.defaults());
        RebuiltField.build(arena.world());
        drive = new SwerveDriveSim(arena.world(), arena.battery(), cfg, Pose3.ofXYYaw(START_X, y, 0, 0));
        arena.addParticipant(drive);
        arena.addLoad(drive);
        arena.advance(0.3);
        double startX = drive.pose().position().x();
        double wheel0 = drive.wheelPositionRad(0);
        for (int i = 0; i < 4; i++) {
            drive.setModuleInputs(i, volts, false, 0, false);
        }
        double t = 0;
        double up = 0;
        double down = 0;
        while (drive.pose().position().x() < END_X && t < timeout) {
            arena.step();
            t += arena.config().physicsDt();
            double pitch = pitchDeg(drive.pose());
            up = Math.max(up, -pitch);
            down = Math.max(down, pitch);
        }
        double wheel = (drive.wheelPositionRad(0) - wheel0) * cfg.modules().get(0).wheelRadius();
        return new Run(t, up, down, wheel, drive.pose().position().x() - startX);
    }

    private static double pitchDeg(Pose3 p) {
        Vec3 forward = p.transformVector(new Vec3(1, 0, 0));
        return Math.toDegrees(Math.asin(-forward.z()));
    }

    @Test
    void suspendedRobotClimbsOverBumpAndLandsLevel() {
        Run r = drive(TestConfigs.suspended(), BUMP_LANE_Y, 6, 6);
        System.out.printf("bump: %.2f s, nose up %.1f°, nose down %.1f°, wheel %.3f m over %.3f m%n", r.seconds(),
                r.maxNoseUpDeg(), r.maxNoseDownDeg(), r.wheelDistance(), r.groundDistance());
        assertTrue(drive.pose().position().x() >= END_X, "stuck at x " + drive.pose().position().x());
        assertTrue(r.maxNoseUpDeg() > 8, "nose up " + r.maxNoseUpDeg());
        assertTrue(r.maxNoseDownDeg() > 8, "nose down " + r.maxNoseDownDeg());
        assertEquals(BUMP_LANE_Y, drive.pose().position().y(), 0.15);

        arena.advance(0.5);
        assertEquals(0, pitchDeg(drive.pose()), 1);
        for (int i = 0; i < 4; i++) {
            assertTrue(drive.normalLoad(i) > 0.5 * 50 * 9.81 / 4, "module " + i + " " + drive.normalLoad(i));
        }
    }

    @Test
    void bumpIsSlowerThanTrenchAndOdometryOverReads() {
        Run bump = drive(TestConfigs.suspended(), BUMP_LANE_Y, 6, 6);
        arena.close();
        Run flat = drive(TestConfigs.suspended(), TRENCH_LANE_Y, 6, 6);
        System.out.printf("bump %.2f s, trench %.2f s; wheel/ground bump %.4f, trench %.4f%n", bump.seconds(),
                flat.seconds(), bump.wheelDistance() / bump.groundDistance(),
                flat.wheelDistance() / flat.groundDistance());
        assertTrue(flat.maxNoseUpDeg() < 2, "trench pitch " + flat.maxNoseUpDeg());
        assertTrue(bump.seconds() > flat.seconds() + 0.05, "bump " + bump.seconds() + " flat " + flat.seconds());
        assertTrue(bump.wheelDistance() / bump.groundDistance() > flat.wheelDistance() / flat.groundDistance() + 0.01);
    }

    /** A 27.5 in wheelbase straddling the ridge needs about 4 in of clearance; with 1 in a crawl high-centers. */
    @Test
    void slowCrawlHighCentersOnRidge() {
        drive(TestConfigs.suspended(), BUMP_LANE_Y, 2, 5);
        double x = drive.pose().position().x();
        assertTrue(x > 4.3 && x < 4.8, "x " + x);
        assertTrue(drive.pose().position().z() > 0.08, "z " + drive.pose().position().z());
    }

    @Test
    void planarRobotIsStoppedByBump() {
        drive(TestConfigs.maxSwerve(), BUMP_LANE_Y, 6, 3);
        assertTrue(drive.pose().position().x() < 4.1, "x " + drive.pose().position().x());
    }
}
