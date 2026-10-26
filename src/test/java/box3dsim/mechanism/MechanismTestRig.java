package box3dsim.mechanism;

import box3dsim.arena.Arena;
import box3dsim.arena.ArenaConfig;
import box3dsim.drive.SwerveDriveSim;
import box3dsim.drive.TestConfigs;
import box3dsim.gamepiece.GamePieceType;
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

/** Arena with a floor and a stationary swerve chassis at the origin, facing +x. */
final class MechanismTestRig implements AutoCloseable {
    static final GamePieceType FUEL =
            new GamePieceType("Fuel", 0.075, 0.215, new Material(0.6, 0.35, 0.05), 0.5, 0.6, 0.3, 0.5, 0, 0);

    final Arena arena = new Arena(ArenaConfig.defaults());
    final SwerveDriveSim drive;

    MechanismTestRig() {
        BodyHandle ground = arena.world().createBody(BodySpec.fixed(Pose3.ofXYYaw(0, 0, -0.5, 0)));
        arena.world().addShape(ground, new ShapeSpec.Box(new Vec3(50, 50, 0.5), Pose3.IDENTITY),
                ShapeOptions.solid(new Material(0.8, 0, 0), 0, CollisionLayers.FIELD));
        drive = new SwerveDriveSim(arena.world(), arena.battery(), TestConfigs.maxSwerve(), Pose3.IDENTITY);
        arena.addParticipant(drive);
        arena.addLoad(drive);
    }

    static RollerConfig roller(double gearing, double radius, double sign) {
        return new RollerConfig(LumpedMechanism.Config.of(DcMotor.neo550(1), gearing, 5e-4), radius, sign, 0.7, 0.8);
    }

    @Override
    public void close() {
        arena.close();
    }
}
