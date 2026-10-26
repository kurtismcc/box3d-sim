package box3dsim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ScaffoldTest {
    @Test
    void runsOnJava25OrNewer() {
        assertTrue(Runtime.version().feature() >= 25);
    }
}
