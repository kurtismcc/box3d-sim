package box3dsim.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.motor.DcMotor;
import box3dsim.motor.LumpedMechanism;
import box3dsim.world.BodyHandle;
import box3dsim.world.BodySpec;
import box3dsim.world.CollisionLayers;
import box3dsim.world.Material;
import box3dsim.world.ShapeOptions;
import box3dsim.world.ShapeSpec;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterSimTest {
    private static final double J = 0.004;
    private static final double RADIUS = 0.0508;

    private MechanismTestRig rig;
    private ShooterSim shooter;

    @BeforeEach
    void setUp() {
        rig = new MechanismTestRig();
        ShooterConfig cfg = new ShooterConfig(
                LumpedMechanism.Config.of(DcMotor.neoVortex(1), 1, J).withFriction(0.05, 1e-4),
                RADIUS, -1, 0.48, 0.8, 0.7, 2.0,
                LumpedMechanism.Config.of(DcMotor.neo550(1), 180, 0.01),
                p -> Math.toRadians(60),
                new Vec3(0.1, 0, 0.64), 0,
                MechanismTestRig.FUEL);
        shooter = new ShooterSim(rig.arena, rig.drive, rig.arena.battery(), cfg, (piece, pose, v) -> {});
        rig.arena.addParticipant(shooter);
        rig.arena.addLoad(shooter);
    }

    @AfterEach
    void tearDown() {
        rig.close();
    }

    /** Runs the flywheel to steady state (τ ≈ J·R·Kv/Kt ≈ 0.8 s, so 6 s is over 7 τ). */
    private double spinUp() {
        shooter.setFlywheelInput(-12, false);
        rig.arena.advance(6);
        return shooter.flywheel().velocity();
    }

    @Test
    void launchSpeedAndPitchFollowFlywheelAndHood() {
        double w = spinUp();
        assertTrue(shooter.launch(MechanismTestRig.FUEL));
        Vec3 v = shooter.lastExitVelocity().orElseThrow();
        assertEquals(0.48 * Math.abs(w) * RADIUS, v.norm(), 0.48 * Math.abs(w) * RADIUS * 0.02);
        assertEquals(60, Math.toDegrees(Math.atan2(v.z(), Math.hypot(v.x(), v.y()))), 0.5);
        assertEquals(1, rig.arena.gamePieces().count(MechanismTestRig.FUEL));
    }

    @Test
    void ceilingOverTheExitBlocksTheShot() {
        BodyHandle arm = rig.arena.world().createBody(BodySpec.fixed(Pose3.ofXYYaw(0, 0, 0.80, 0)));
        rig.arena.world().addShape(arm, new ShapeSpec.Box(new Vec3(1, 1, 0.2348), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.4, 0.3, 0), 0, CollisionLayers.FIELD));
        spinUp();
        assertFalse(shooter.launch(MechanismTestRig.FUEL));
        assertTrue(shooter.exitBlocked());
        assertEquals(0, shooter.shotsFired());
        assertEquals(0, rig.arena.gamePieces().count(MechanismTestRig.FUEL));
    }

    @Test
    void flywheelLosesShotEnergy() {
        double w0 = spinUp();
        shooter.launch(MechanismTestRig.FUEL);
        double v = shooter.lastExitVelocity().orElseThrow().norm();
        double m = MechanismTestRig.FUEL.mass();
        double r = MechanismTestRig.FUEL.radius();
        double spin = 0.8 * v / r;
        double energy = 0.5 * m * v * v + 0.5 * 0.4 * m * r * r * spin * spin;
        double expected = Math.sqrt(w0 * w0 - 2 * energy / 0.7 / J);
        assertEquals(expected, Math.abs(shooter.flywheel().velocity()), expected * 0.01);
    }

    @Test
    void flywheelRecoversAfterBurst() {
        double w0 = Math.abs(spinUp());
        for (int i = 0; i < 5; i++) {
            shooter.launch(MechanismTestRig.FUEL);
            rig.arena.advance(0.1);
        }
        double lowest = Math.abs(shooter.flywheel().velocity());
        rig.arena.advance(1);
        double recovered = Math.abs(shooter.flywheel().velocity());
        System.out.printf("flywheel %.0f rpm, after 5-shot burst %.0f rpm, 1 s later %.0f rpm%n",
                w0 * 60 / (2 * Math.PI), lowest * 60 / (2 * Math.PI), recovered * 60 / (2 * Math.PI));
        assertTrue(lowest < w0 * 0.97);
        assertEquals(w0, recovered, w0 * 0.05);
    }

    @Test
    void movingRobotAddsItsVelocity() {
        spinUp();
        for (int i = 0; i < 4; i++) {
            rig.drive.setModuleInputs(i, 0, true, 0, false);
        }
        rig.arena.world().setVelocity(rig.drive.body(), new Vec3(2, 0, 0), Vec3.ZERO);
        rig.arena.step();
        double chassisVx = rig.drive.fieldVelocity().x();
        assertTrue(chassisVx > 1.5, "chassis vx " + chassisVx);
        double exitSpeed = 0.48 * Math.abs(shooter.flywheel().velocity()) * RADIUS;
        shooter.launch(MechanismTestRig.FUEL);
        double vx = shooter.lastExitVelocity().orElseThrow().x();
        assertEquals(chassisVx + exitSpeed * Math.cos(Math.toRadians(60)), vx, 0.05);
    }

    @Test
    void refusesWhenFlywheelIsStopped() {
        assertFalse(shooter.launch(MechanismTestRig.FUEL));
        assertEquals(0, shooter.shotsFired());
    }
}
