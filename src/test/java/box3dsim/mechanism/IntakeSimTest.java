package box3dsim.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.gamepiece.GamePieceType;
import box3dsim.math.Vec3;
import box3dsim.motor.DcMotor;
import box3dsim.motor.LumpedMechanism;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeSimTest {
    private MechanismTestRig rig;
    private CountingSink sink;
    private IntakeSim intake;

    private static final class CountingSink implements PieceSink {
        int capacity = 100;
        int count;

        @Override
        public boolean canAccept() {
            return count < capacity;
        }

        @Override
        public void accept(GamePieceType type) {
            count++;
        }
    }

    @BeforeEach
    void setUp() {
        rig = new MechanismTestRig();
        sink = new CountingSink();
        IntakeConfig cfg = new IntakeConfig(
                MechanismTestRig.roller(3, 0.03, 1),
                LumpedMechanism.Config.of(DcMotor.neo(1), 50, 0.1).withLimits(-2.0, 0),
                -2.0, 0, 0.9,
                new Vec3(0.55, 0, 0.075), new Vec3(0.10, 0.30, 0.10),
                MechanismTestRig.FUEL);
        intake = new IntakeSim(rig.arena, rig.drive.body(), rig.arena.battery(), cfg, sink);
        rig.arena.addParticipant(intake);
        rig.arena.addLoad(intake);
    }

    @AfterEach
    void tearDown() {
        rig.close();
    }

    private void ball(double y, boolean awake) {
        rig.arena.gamePieces().spawn(MechanismTestRig.FUEL, new Vec3(0.55, y, 0.075), Vec3.ZERO, Vec3.ZERO, awake);
    }

    private void deploy() {
        intake.setPivotInput(-12, false);
        rig.arena.advance(0.5);
        assertEquals(1.0, intake.deployProgress(), 1e-9);
    }

    @Test
    void stowedIntakeTakesNothing() {
        ball(0, true);
        intake.setRollerInput(12, false);
        rig.arena.advance(1);
        assertEquals(0, sink.count);
    }

    @Test
    void deployedSpinningIntakeTakesPiece() {
        ball(0, true);
        deploy();
        intake.setRollerInput(12, false);
        rig.arena.advance(0.5);
        assertEquals(1, sink.count);
        assertEquals(0, rig.arena.gamePieces().count(MechanismTestRig.FUEL));
    }

    @Test
    void fasterRollerTakesPiecesFaster() {
        double slow = timeToTakeFour(2);
        tearDown();
        setUp();
        double fast = timeToTakeFour(12);
        assertTrue(fast < slow / 2, "fast " + fast + " slow " + slow);
    }

    private double timeToTakeFour(double volts) {
        for (double y : new double[] {-0.225, -0.075, 0.075, 0.225}) {
            ball(y, true);
        }
        deploy();
        intake.setRollerInput(volts, false);
        double start = rig.arena.time();
        while (sink.count < 4 && rig.arena.time() - start < 5) {
            rig.arena.step();
        }
        assertEquals(4, sink.count);
        return rig.arena.time() - start;
    }

    @Test
    void stopsAtSinkCapacity() {
        sink.capacity = 3;
        for (double y : new double[] {-0.225, -0.075, 0.075, 0.225}) {
            ball(y, true);
        }
        deploy();
        intake.setRollerInput(12, false);
        rig.arena.advance(1);
        assertEquals(3, sink.count);
    }

    @Test
    void takesSleepingPieces() {
        ball(0, false);
        deploy();
        intake.setRollerInput(12, false);
        rig.arena.advance(0.5);
        assertEquals(1, sink.count);
    }
}
