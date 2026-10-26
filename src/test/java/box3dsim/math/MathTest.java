package box3dsim.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class MathTest {
    private static final double EPS = 1e-9;

    private static void assertVec(Vec3 expected, Vec3 actual) {
        assertEquals(expected.x(), actual.x(), EPS);
        assertEquals(expected.y(), actual.y(), EPS);
        assertEquals(expected.z(), actual.z(), EPS);
    }

    @Test
    void yawRotatesXIntoY() {
        assertVec(new Vec3(0, 1, 0), Quat.fromYaw(Math.PI / 2).rotate(new Vec3(1, 0, 0)));
    }

    @Test
    void rollPitchYawKeepsYaw() {
        for (double pitch : new double[] {-1.2, -0.3, 0, 0.4, 1.3}) {
            Quat q = Quat.fromRollPitchYaw(0.7, pitch, 2.1);
            assertEquals(2.1, q.yaw(), EPS);
        }
    }

    @Test
    void poseComposition() {
        Pose3 a = Pose3.ofXYYaw(1, 2, 0, Math.PI / 2);
        Pose3 b = Pose3.ofXYYaw(1, 0, 0, Math.PI / 2);
        Pose3 c = a.times(b);
        assertVec(new Vec3(1, 3, 0), c.position());
        assertEquals(Math.PI, Math.abs(c.rotation().yaw()), EPS);
    }

    @Test
    void inverseTransformRoundTrip() {
        Pose3 p = new Pose3(new Vec3(3, -1, 0.5), Quat.fromRollPitchYaw(0.2, -0.4, 1.1));
        Vec3 v = new Vec3(0.3, 0.7, -2.0);
        assertVec(v, p.inverseTransformPoint(p.transformPoint(v)));
    }
}
