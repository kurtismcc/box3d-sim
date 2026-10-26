package box3dsim.world;

import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import java.util.List;

/** Collision shape geometry in body-local coordinates. */
public sealed interface ShapeSpec {
    record Sphere(Vec3 center, double radius) implements ShapeSpec {}

    /** Segment p1–p2 swept by radius. */
    record Capsule(Vec3 p1, Vec3 p2, double radius) implements ShapeSpec {}

    record Box(Vec3 halfExtents, Pose3 localPose) implements ShapeSpec {}

    /** Convex hull of up to 128 points. */
    record Hull(List<Vec3> points, Pose3 localPose) implements ShapeSpec {}

    /** Triangle mesh, counter-clockwise winding. Static bodies only. */
    record Mesh(List<Vec3> vertices, int[] triangleIndices) implements ShapeSpec {}
}
