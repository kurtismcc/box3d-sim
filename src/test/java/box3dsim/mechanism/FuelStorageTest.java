package box3dsim.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FuelStorageTest {
    private MechanismTestRig rig;
    private final List<Double> launches = new ArrayList<>();
    private boolean accept = true;
    private FuelStorage storage;

    @BeforeEach
    void setUp() {
        rig = new MechanismTestRig();
        StorageConfig cfg = new StorageConfig(40, 8, MechanismTestRig.roller(5, 0.025, -1),
                MechanismTestRig.roller(3, 0.025, -1), MechanismTestRig.FUEL);
        storage = new FuelStorage(rig.arena.battery(), cfg, type -> {
            if (accept) {
                launches.add(rig.arena.time());
            }
            return accept;
        });
        rig.arena.addParticipant(storage);
        rig.arena.addLoad(storage);
        rig.arena.reset();
    }

    @AfterEach
    void tearDown() {
        rig.close();
    }

    @Test
    void feedsEveryPreloadedPiece() {
        storage.setIndexerInput(-8, false);
        storage.setFeederInput(-11, false);
        rig.arena.advance(3);
        assertEquals(8, launches.size());
        assertEquals(0, storage.totalCount());
        double minGap = Double.MAX_VALUE;
        for (int i = 1; i < launches.size(); i++) {
            minGap = Math.min(minGap, launches.get(i) - launches.get(i - 1));
        }
        assertTrue(minGap > 0.004, "pieces must not teleport through: gap " + minGap);
    }

    @Test
    void refusedLaunchKeepsPieceStaged() {
        accept = false;
        storage.setIndexerInput(-8, false);
        storage.setFeederInput(-11, false);
        rig.arena.advance(1);
        assertTrue(storage.staged());
        assertEquals(7, storage.hopperCount());
    }

    @Test
    void reversingFeederUnstages() {
        accept = false;
        storage.setIndexerInput(-8, false);
        storage.setFeederInput(-11, false);
        rig.arena.advance(1);
        storage.setIndexerInput(0, false);
        storage.setFeederInput(11, false);
        rig.arena.advance(0.5);
        assertFalse(storage.staged());
        assertEquals(8, storage.hopperCount());
    }
}
