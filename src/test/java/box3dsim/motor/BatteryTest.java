package box3dsim.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class BatteryTest {
    @Test
    void noLoadKeepsOpenCircuitVoltage() {
        Battery b = Battery.defaults();
        b.update(0, 0.004);
        assertEquals(12.6, b.voltage(), 1e-12);
        assertFalse(b.isBrownedOut());
    }

    @Test
    void steadyLoadSags() {
        Battery b = Battery.defaults();
        for (int i = 0; i < 250; i++) {
            b.update(100, 0.004);
        }
        assertEquals(10.6, b.voltage(), 0.01);
    }

    @Test
    void heavyLoadClampsAtBrownout() {
        Battery b = Battery.defaults();
        for (int i = 0; i < 250; i++) {
            b.update(1000, 0.004);
        }
        assertEquals(6.8, b.voltage(), 1e-12);
        assertTrue(b.isBrownedOut());
    }
}
