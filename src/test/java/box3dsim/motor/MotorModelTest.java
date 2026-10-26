package box3dsim.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MotorModelTest {
    private static final double DT = 0.004;

    private static Battery stiff() {
        return new Battery(12, 0, 0, 1e-9);
    }

    private static void run(LumpedMechanism m, double seconds) {
        for (int i = 0; i < Math.round(seconds / DT); i++) {
            m.step(DT);
        }
    }

    @Test
    void vortexConstants() {
        DcMotor v = DcMotor.neoVortex(1);
        assertEquals(12.0 / 211, v.resistance(), 1e-9);
        assertEquals(3.60 / 211, v.kt(), 1e-9);
        assertEquals(6784 * 2 * Math.PI / 60 / (12 - 12.0 / 211 * 3.6), v.kv(), 1e-9);
        assertEquals(60.2, v.kv(), 0.1);
    }

    @Test
    void reachesNoLoadSpeed() {
        DcMotor v = DcMotor.neoVortex(1);
        LumpedMechanism m = new LumpedMechanism(LumpedMechanism.Config.of(v, 1, 1e-3), stiff());
        m.setInput(12, false);
        run(m, 2);
        assertEquals(v.kv() * 12, m.rotorVelocity(), v.kv() * 12 * 0.01);
    }

    @Test
    void timeConstantMatchesModel() {
        DcMotor v = DcMotor.neoVortex(1);
        double j = 1e-3;
        double tau = j * v.resistance() * v.kv() / v.kt();
        LumpedMechanism m = new LumpedMechanism(LumpedMechanism.Config.of(v, 1, j), stiff());
        m.setInput(12, false);
        double target = 0.632 * v.kv() * 12;
        double t = 0;
        while (m.velocity() < target) {
            m.step(DT);
            t += DT;
        }
        assertEquals(tau, t, tau * 0.05);
    }

    @Test
    void tinyInertiaStaysStable() {
        DcMotor n = DcMotor.neo550(1);
        LumpedMechanism m = new LumpedMechanism(LumpedMechanism.Config.of(n, 50, 1e-6), stiff());
        m.setInput(12, false);
        double limit = n.kv() * 12 / 50 * 1.001;
        for (int i = 0; i < 1000; i++) {
            m.step(DT);
            assertTrue(m.velocity() <= limit, "overshoot " + m.velocity());
            assertTrue(m.velocity() >= 0, "negative " + m.velocity());
        }
    }

    @Test
    void brakeStopsFasterThanCoast() {
        DcMotor v = DcMotor.neoVortex(1);
        LumpedMechanism.Config cfg = LumpedMechanism.Config.of(v, 1, 1e-3).withFriction(0.01, 0);
        LumpedMechanism coast = new LumpedMechanism(cfg, stiff());
        LumpedMechanism brake = new LumpedMechanism(cfg, stiff());
        double w0 = v.kv() * 12;
        coast.setState(0, w0);
        brake.setState(0, w0);
        coast.setInput(0, true);
        brake.setInput(0, false);
        run(coast, 0.2);
        run(brake, 0.2);
        assertEquals(w0 - 0.01 / 1e-3 * 0.2, coast.velocity(), 0.5);
        assertTrue(brake.velocity() < 0.4 * w0, "brake " + brake.velocity());
        assertTrue(brake.velocity() < coast.velocity() / 2);
        assertEquals(0, coast.statorCurrent());
    }

    @Test
    void currentLimitHoldsAtStall() {
        LumpedMechanism m = new LumpedMechanism(
                LumpedMechanism.Config.of(DcMotor.neoVortex(1), 1, 1e3).withCurrentLimit(40), stiff());
        m.setInput(12, false);
        run(m, 0.1);
        assertEquals(40, m.statorCurrent(), 0.5);
    }

    @Test
    void hardLimitStopsMotion() {
        LumpedMechanism m = new LumpedMechanism(
                LumpedMechanism.Config.of(DcMotor.neo(1), 10, 1e-2).withLimits(Double.NEGATIVE_INFINITY, 1.0),
                stiff());
        m.setInput(12, false);
        run(m, 2);
        assertEquals(1.0, m.position(), 1e-12);
        assertEquals(0.0, m.velocity(), 1e-12);
    }

    @Test
    void removeEnergyFollowsKineticEnergy() {
        LumpedMechanism m = new LumpedMechanism(LumpedMechanism.Config.of(DcMotor.neoVortex(1), 1, 0.002), stiff());
        m.setState(0, 300);
        m.removeEnergy(10);
        assertEquals(Math.sqrt(90000 - 10000), m.velocity(), 1e-9);
    }
}
