package box3dsim.season.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;

import box3dsim.arena.Alliance;
import box3dsim.arena.Arena;
import box3dsim.arena.ArenaConfig;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class FieldResetTest {
    @Test
    void resetRestoresScatteredFuel() {
        try (Arena arena = new Arena(ArenaConfig.defaults())) {
            RebuiltMatch match = new RebuiltMatch(arena, RebuiltField.build(arena.world()), RebuiltMatch.Config.defaults());
            arena.addParticipant(match);
            arena.reset();
            // Scatter the neutral zone with a heavy ball fired through it.
            arena.gamePieces().spawn(RebuiltFuel.TYPE, new Vec3(6.0, 4.0, 0.075), new Vec3(15, 0.5, 0), Vec3.ZERO, true);
            arena.advance(2);
            assertEquals(true, startPositionsMatched(match) < 408, "the scatter should move some FUEL");

            arena.reset();
            assertEquals(408, startPositionsMatched(match));
            assertEquals(456, match.fuelDisplayPoses().size());
            arena.advance(0.5);
            assertEquals(408, startPositionsMatched(match));
        }
    }

    private static int startPositionsMatched(RebuiltMatch match) {
        List<Vec3> expected = new ArrayList<>(RebuiltFuelLayout.neutralZone());
        for (Alliance a : Alliance.values()) {
            expected.addAll(RebuiltFuelLayout.depot(a));
        }
        List<Pose3> poses = match.fuelDisplayPoses();
        int matched = 0;
        for (Vec3 e : expected) {
            for (Pose3 p : poses) {
                if (p.position().minus(e).norm() < 1e-3) {
                    matched++;
                    break;
                }
            }
        }
        return matched;
    }
}
