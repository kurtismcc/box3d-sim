package box3dsim.math;

/** Immutable rotation quaternion, scalar first. */
public record Quat(double w, double x, double y, double z) {
    public static final Quat IDENTITY = new Quat(1, 0, 0, 0);

    public static Quat fromAxisAngle(Vec3 axis, double angle) {
        Vec3 a = axis.normalized();
        double s = Math.sin(angle / 2);
        return new Quat(Math.cos(angle / 2), a.x() * s, a.y() * s, a.z() * s);
    }

    /** Rotation about +z. */
    public static Quat fromYaw(double yaw) {
        return new Quat(Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2));
    }

    /** Extrinsic X then Y then Z rotation, matching WPILib Rotation3d(roll, pitch, yaw). */
    public static Quat fromRollPitchYaw(double roll, double pitch, double yaw) {
        Quat qx = new Quat(Math.cos(roll / 2), Math.sin(roll / 2), 0, 0);
        Quat qy = new Quat(Math.cos(pitch / 2), 0, Math.sin(pitch / 2), 0);
        Quat qz = fromYaw(yaw);
        return qz.times(qy).times(qx);
    }

    /** Hamilton product this * o. */
    public Quat times(Quat o) {
        return new Quat(
                w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w);
    }

    /** Conjugate of the normalized quaternion. */
    public Quat inverse() {
        Quat n = normalized();
        return new Quat(n.w, -n.x, -n.y, -n.z);
    }

    public Vec3 rotate(Vec3 v) {
        Vec3 u = new Vec3(x, y, z);
        Vec3 t = u.cross(v).times(2);
        return v.plus(t.times(w)).plus(u.cross(t));
    }

    public double yaw() {
        return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }

    public Quat normalized() {
        double n = Math.sqrt(w * w + x * x + y * y + z * z);
        return n == 0 ? IDENTITY : new Quat(w / n, x / n, y / n, z / n);
    }
}
