package box3dsim.season.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaConfig;
import box3dsim.drive.SwerveDriveSim;
import box3dsim.drive.TestConfigs;
import box3dsim.gamepiece.GamePiece;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.mechanism.ShooterConfig;
import box3dsim.mechanism.ShooterSim;
import box3dsim.motor.DcMotor;
import box3dsim.motor.LumpedMechanism;
import java.util.Comparator;
import java.util.Locale;
import org.junit.jupiter.api.Test;

/**
 * Report, more than assertion: where the 2026-Offseason presets put FUEL relative to the blue HUB opening. Shooter
 * constants are copied from the robot repo's SimConstants (keep them in sync when calibrating).
 */
class ShotCalibrationTest {
    private static final double HUB_X = 4.6255;
    private static final double HUB_Y = 4.0346;
    private static final double OPENING_Z = 1.829;
    private static final double EXIT_AHEAD = 0.10;

    private record Preset(String name, double rpm, double hoodMotorRotations) {}

    private record Result(double crossX, double crossY, boolean inOpening, double apexZ) {}

    private static ShooterConfig shooterConfig() {
        LumpedMechanism.Config flywheel =
                LumpedMechanism.Config.of(DcMotor.neoVortex(1), 1, 0.004).withFriction(0.05, 1e-4);
        LumpedMechanism.Config hood = LumpedMechanism.Config.of(DcMotor.neo550(1), 180, 0.01).withFriction(0.2, 0);
        return new ShooterConfig(flywheel, 0.0508, -1, 0.48, 0.8, 0.7, 2.0, hood, p -> Math.toRadians(72) + p,
                new Vec3(EXIT_AHEAD, 0, 0.64), 0, RebuiltFuel.TYPE);
    }

    private static Result shoot(Preset preset, double distance) {
        try (Arena arena = new Arena(ArenaConfig.defaults())) {
            RebuiltField.build(arena.world());
            Pose3 start = Pose3.ofXYYaw(HUB_X - distance - EXIT_AHEAD, HUB_Y, 0, 0);
            SwerveDriveSim drive = new SwerveDriveSim(arena.world(), arena.battery(), TestConfigs.maxSwerve(), start);
            ShooterSim shooter = new ShooterSim(arena, drive, arena.battery(), shooterConfig(), (piece, p, v) -> {});
            arena.addParticipant(drive);
            arena.addParticipant(shooter);
            arena.step();
            shooter.flywheel().setState(0, -preset.rpm() * 2 * Math.PI / 60);
            shooter.hood().setState(preset.hoodMotorRotations() * 2 * Math.PI / 180, 0);
            assertTrue(shooter.launch(RebuiltFuel.TYPE));
            GamePiece ball = arena.gamePieces().all().stream()
                    .max(Comparator.comparingInt(GamePiece::id)).orElseThrow();

            double apex = 0;
            Vec3 prev = ball.pose().position();
            for (int i = 0; i < 1000 && ball.isAlive(); i++) {
                arena.step();
                Vec3 pos = ball.pose().position();
                apex = Math.max(apex, pos.z());
                boolean descendingThrough = prev.z() > OPENING_Z && pos.z() <= OPENING_Z;
                boolean landed = pos.z() < RebuiltFuel.TYPE.radius() + 0.01;
                if (descendingThrough || landed) {
                    boolean in = Math.abs(pos.x() - HUB_X) < 0.53 && Math.abs(pos.y() - HUB_Y) < 0.53 && descendingThrough;
                    return new Result(pos.x(), pos.y(), in, apex);
                }
                prev = pos;
            }
            // Removed by the HUB sensor or still flying: report the last known position.
            return new Result(prev.x(), prev.y(), !ball.isAlive(), apex);
        }
    }

    @Test
    void reportPresetsAgainstTheHub() {
        Preset hub = new Preset("hub", 2700, 0);
        Preset side = new Preset("side", 3000, -8.5);
        StringBuilder table = new StringBuilder("preset  d     cross_x  cross_y  in_opening  apex_z\n");
        for (Preset p : new Preset[] {hub, side}) {
            for (double d : new double[] {1.0, 1.5, 2.0, 2.5, 3.0}) {
                Result r = shoot(p, d);
                table.append(String.format(Locale.ROOT, "%-6s  %.2f  %7.3f  %7.3f  %-10s  %.2f%n", p.name(), d,
                        r.crossX(), r.crossY(), r.inOpening(), r.apexZ()));
            }
        }
        System.out.print(table);
        // Shoot at Hub auto end point: robot center at x = 2.75, exit 0.10 ahead of it.
        Result auto = shoot(hub, HUB_X - 2.75 - EXIT_AHEAD);
        assertTrue(auto.inOpening(), "hub preset from the auto end point misses: " + auto);
    }
}
