package box3dsim.math;

/** Immutable 3D vector. */
public record Vec3(double x, double y, double z) {
    public static final Vec3 ZERO = new Vec3(0, 0, 0);

    public Vec3 plus(Vec3 o) {
        return new Vec3(x + o.x, y + o.y, z + o.z);
    }

    public Vec3 minus(Vec3 o) {
        return new Vec3(x - o.x, y - o.y, z - o.z);
    }

    public Vec3 times(double s) {
        return new Vec3(x * s, y * s, z * s);
    }

    public double dot(Vec3 o) {
        return x * o.x + y * o.y + z * o.z;
    }

    public Vec3 cross(Vec3 o) {
        return new Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }

    public double norm() {
        return Math.sqrt(normSquared());
    }

    public double normSquared() {
        return x * x + y * y + z * z;
    }

    /** Unit vector in the same direction; ZERO stays ZERO. */
    public Vec3 normalized() {
        double n = norm();
        return n == 0 ? ZERO : times(1.0 / n);
    }
}
