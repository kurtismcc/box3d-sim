package box3dsim.arena;

import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.drive.SwerveDriveSim;
import box3dsim.drive.TestConfigs;
import box3dsim.math.Pose3;
import org.junit.jupiter.api.Test;

class ArenaTest {
    @Test
    void swerveDrivesThroughArena() {
        try (Arena arena = new Arena(ArenaConfig.defaults())) {
            SwerveDriveSim drive = new SwerveDriveSim(arena.world(), arena.battery(), TestConfigs.maxSwerve(),
                    Pose3.IDENTITY);
            arena.addParticipant(drive);
            arena.addLoad(drive);
            for (int i = 0; i < 4; i++) {
                drive.setModuleInputs(i, 12, false, 0, false);
            }
            arena.advance(3);
            assertTrue(drive.fieldVelocity().x() > 6.5, "vx " + drive.fieldVelocity().x());
            assertTrue(arena.battery().voltage() < 12.6);
            assertTrue(Math.abs(arena.time() - 3.0) < 1e-9);
        }
    }
}
