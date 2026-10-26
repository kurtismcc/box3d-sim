package box3dsim.world;

import box3dsim.math.Pose3;
import box3dsim.math.Vec3;

/** Body creation parameters. */
public record BodySpec(
        BodyType type,
        Pose3 pose,
        Vec3 linearVelocity,
        Vec3 angularVelocity,
        double linearDamping,
        double angularDamping,
        double gravityScale,
        boolean bullet,
        boolean enableSleep,
        boolean awake,
        MotionLocks locks,
        String name) {

    public static BodySpec dynamic(Pose3 pose) {
        return new BodySpec(
                BodyType.DYNAMIC, pose, Vec3.ZERO, Vec3.ZERO, 0, 0, 1, false, true, true, MotionLocks.NONE, null);
    }

    public static BodySpec fixed(Pose3 pose) {
        return new BodySpec(
                BodyType.STATIC, pose, Vec3.ZERO, Vec3.ZERO, 0, 0, 1, false, true, true, MotionLocks.NONE, null);
    }

    public BodySpec withLocks(MotionLocks l) {
        return new BodySpec(type, pose, linearVelocity, angularVelocity, linearDamping, angularDamping, gravityScale,
                bullet, enableSleep, awake, l, name);
    }

    public BodySpec withBullet(boolean b) {
        return new BodySpec(type, pose, linearVelocity, angularVelocity, linearDamping, angularDamping, gravityScale,
                b, enableSleep, awake, locks, name);
    }

    /** false = created asleep, for resting game pieces at match start. */
    public BodySpec withAwake(boolean a) {
        return new BodySpec(type, pose, linearVelocity, angularVelocity, linearDamping, angularDamping, gravityScale,
                bullet, enableSleep, a, locks, name);
    }

    public BodySpec withSleep(boolean enable) {
        return new BodySpec(type, pose, linearVelocity, angularVelocity, linearDamping, angularDamping, gravityScale,
                bullet, enable, awake, locks, name);
    }

    public BodySpec withGravityScale(double g) {
        return new BodySpec(type, pose, linearVelocity, angularVelocity, linearDamping, angularDamping, g, bullet,
                enableSleep, awake, locks, name);
    }

    public BodySpec withDamping(double lin, double ang) {
        return new BodySpec(type, pose, linearVelocity, angularVelocity, lin, ang, gravityScale, bullet, enableSleep,
                awake, locks, name);
    }

    public BodySpec withVelocity(Vec3 lin, Vec3 ang) {
        return new BodySpec(type, pose, lin, ang, linearDamping, angularDamping, gravityScale, bullet, enableSleep,
                awake, locks, name);
    }

    public BodySpec withName(String n) {
        return new BodySpec(type, pose, linearVelocity, angularVelocity, linearDamping, angularDamping, gravityScale,
                bullet, enableSleep, awake, locks, n);
    }
}
