package box3dsim.math;

/** Rigid transform: rotation, then translation. */
public record Pose3(Vec3 position, Quat rotation) {
    public static final Pose3 IDENTITY = new Pose3(Vec3.ZERO, Quat.IDENTITY);

    public static Pose3 ofXYYaw(double x, double y, double z, double yaw) {
        return new Pose3(new Vec3(x, y, z), Quat.fromYaw(yaw));
    }

    public Vec3 transformPoint(Vec3 local) {
        return rotation.rotate(local).plus(position);
    }

    public Vec3 transformVector(Vec3 local) {
        return rotation.rotate(local);
    }

    public Vec3 inverseTransformPoint(Vec3 world) {
        return rotation.inverse().rotate(world.minus(position));
    }

    /** Composition this ∘ local. */
    public Pose3 times(Pose3 local) {
        return new Pose3(transformPoint(local.position), rotation.times(local.rotation));
    }
}
