package box3dsim.season.rebuilt2026;

import box3dsim.arena.Alliance;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import box3dsim.world.BodyHandle;
import box3dsim.world.BodySpec;
import box3dsim.world.CollisionLayers;
import box3dsim.world.Material;
import box3dsim.world.ShapeHandle;
import box3dsim.world.ShapeOptions;
import box3dsim.world.ShapeSpec;
import box3dsim.world.SimWorld;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Static 2026 REBUILT welded field in the WPILib blue-origin frame. Geometry is written for blue; red is the same
 * geometry rotated 180° about the field center. Sources and assumptions: docs/steps/08-rebuilt-field.md.
 */
public final class RebuiltField {
    public static final double LENGTH = 16.541;
    public static final double WIDTH = 8.069;
    /** Center of the blue HUB's top opening. */
    public static final Vec3 BLUE_HUB_CENTER = new Vec3(4.6255, 4.0346, 1.829);
    /** Blue ALLIANCE ZONE is x ≤ this (158.6 in). */
    public static final double STARTING_LINE_X = 4.0284;
    public static final double CENTER_LINE_X = LENGTH / 2;

    // HUB (47 in square, 72 in opening). ASSUMPTION: solid base up to 1.40 m, square rim instead of the hex.
    private static final double HUB_MIN_X = 4.0285;
    private static final double HUB_MAX_X = 5.2225;
    private static final double HUB_MIN_Y = 3.4376;
    private static final double HUB_MAX_Y = 4.6316;
    private static final double HUB_BASE_TOP = 1.40;
    private static final double HUB_RIM_TOP = 1.829;
    private static final double HUB_RIM_THICKNESS = 0.064;

    private RebuiltField() {}

    public static Vec3 mirror(Vec3 p) {
        return new Vec3(LENGTH - p.x(), WIDTH - p.y(), p.z());
    }

    public static Vec3 hubCenter(Alliance a) {
        return a == Alliance.BLUE ? BLUE_HUB_CENTER : mirror(BLUE_HUB_CENTER);
    }

    public static boolean inAllianceZone(Alliance a, double x) {
        return a == Alliance.BLUE ? x <= STARTING_LINE_X : x >= LENGTH - STARTING_LINE_X;
    }

    public static RebuiltFieldHandles build(SimWorld world) {
        Builder b = new Builder(world);
        b.box(Alliance.BLUE, -1, -1, -0.1, LENGTH + 1, WIDTH + 1, 0, field(RebuiltMaterials.CARPET));
        b.box(Alliance.BLUE, 0, -0.1, 0, LENGTH, 0, 0.508, field(RebuiltMaterials.POLYCARB));
        b.box(Alliance.BLUE, 0, WIDTH, 0, LENGTH, WIDTH + 0.1, 0.508, field(RebuiltMaterials.POLYCARB));

        Map<Alliance, ShapeHandle> hubSensors = new EnumMap<>(Alliance.class);
        Map<Alliance, ShapeHandle> corralSensors = new EnumMap<>(Alliance.class);
        for (Alliance a : Alliance.values()) {
            buildAllianceSide(b, a);
            hubSensors.put(a, b.sensor(a, 4.13, 3.54, 1.42, 5.12, 4.53, 1.70));
            corralSensors.put(a, b.sensor(a, -0.9, 0.27, 0.0, -0.05, 1.06, 0.20));
        }
        return new RebuiltFieldHandles(hubSensors, corralSensors, List.copyOf(b.bodies));
    }

    private static void buildAllianceSide(Builder b, Alliance a) {
        ShapeOptions poly = field(RebuiltMaterials.POLYCARB);

        // Alliance wall with the CORRAL base opening (32 × 7 in, bottom 1.88 in, centered y = 0.666).
        b.box(a, -0.1, -0.1, 0, 0, 0.2596, 2.0, poly);
        b.box(a, -0.1, 1.0724, 0, 0, WIDTH + 0.1, 2.0, poly);
        b.box(a, -0.1, 0.2596, 0.2256, 0, 1.0724, 2.0, poly);
        b.box(a, -0.1, 0.2596, 0, 0, 1.0724, 0.0478, poly);
        b.box(a, -0.1, 0.645, 0.0478, 0, 0.687, 0.2256, poly);
        // CORRAL pocket behind the wall (35.8 × 37.6 in, 8.13 in panels).
        b.box(a, -0.955, 0.2096, 0, -0.1, 0.2596, 0.2065, poly);
        b.box(a, -0.955, 1.0724, 0, -0.1, 1.1224, 0.2065, poly);
        b.box(a, -1.0, 0.2096, 0, -0.955, 1.1224, 0.2065, poly);

        // HUB base, rim and the back net on the NEUTRAL ZONE face (ASSUMPTION: solid panel).
        b.box(a, HUB_MIN_X, HUB_MIN_Y, 0, HUB_MAX_X, HUB_MAX_Y, HUB_BASE_TOP, poly);
        double t = HUB_RIM_THICKNESS;
        b.box(a, HUB_MIN_X, HUB_MIN_Y, HUB_BASE_TOP, HUB_MIN_X + t, HUB_MAX_Y, HUB_RIM_TOP, poly);
        b.box(a, HUB_MAX_X - t, HUB_MIN_Y, HUB_BASE_TOP, HUB_MAX_X, HUB_MAX_Y, HUB_RIM_TOP, poly);
        b.box(a, HUB_MIN_X + t, HUB_MIN_Y, HUB_BASE_TOP, HUB_MAX_X - t, HUB_MIN_Y + t, HUB_RIM_TOP, poly);
        b.box(a, HUB_MIN_X + t, HUB_MAX_Y - t, HUB_BASE_TOP, HUB_MAX_X - t, HUB_MAX_Y, HUB_RIM_TOP, poly);
        b.box(a, HUB_MAX_X, 3.2926, HUB_RIM_TOP, HUB_MAX_X + 0.02, 4.7766, 3.057, poly);

        // BUMPs: robots drive through them in v0 (PIECE_BLOCKER); FUEL rolls over them.
        ShapeOptions bump = ShapeOptions.solid(RebuiltMaterials.HDPE, 0, CollisionLayers.PIECE_BLOCKER);
        b.prism(a, 4.0615, 5.1895, 4.6255, 0.1654, 1.5834, 3.4376, bump);
        b.prism(a, 4.0615, 5.1895, 4.6255, 0.1654, 4.6316, 6.4856, bump);

        // TRENCH arms (22.25 in clearance) and posts, on both guardrails.
        b.box(a, HUB_MIN_X, 0, 0.5652, HUB_MAX_X, 1.2786, 1.0224, poly);
        b.box(a, HUB_MIN_X, 1.2786, 0, HUB_MAX_X, 1.5834, 1.0224, poly);
        b.box(a, HUB_MIN_X, WIDTH - 1.2786, 0.5652, HUB_MAX_X, WIDTH, 1.0224, poly);
        b.box(a, HUB_MIN_X, WIDTH - 1.5834, 0, HUB_MAX_X, WIDTH - 1.2786, 1.0224, poly);

        // TOWER uprights, rungs and support (ASSUMPTION: support footprint).
        b.box(a, 1.0175, 3.2962, 0, 1.1065, 3.3342, 1.831, poly);
        b.box(a, 1.0175, 4.1535, 0, 1.1065, 4.1915, 1.831, poly);
        for (double z : new double[] {0.6858, 1.1430, 1.6002}) {
            b.capsule(a, new Vec3(1.062, 3.1660, z), new Vec3(1.062, 4.3217, z), 0.0211, poly);
        }
        b.box(a, 0, 3.3342, 0.7214, 1.0175, 4.1535, 1.1019, poly);

        // DEPOT barrier strips (1 in tall, 3 in wide) on the three open sides.
        b.box(a, 0.6477, 5.4296, 0, 0.7239, 6.4964, 0.0254, poly);
        b.box(a, 0, 5.3915, 0, 0.6858, 5.4677, 0.0254, poly);
        b.box(a, 0, 6.4583, 0, 0.6858, 6.5345, 0.0254, poly);
    }

    private static ShapeOptions field(Material m) {
        return ShapeOptions.solid(m, 0, CollisionLayers.FIELD);
    }

    /** Adds static geometry given in blue coordinates, mirroring it for red. */
    private static final class Builder {
        private final SimWorld world;
        private final BodyHandle solids;
        private final List<BodyHandle> bodies = new ArrayList<>();

        Builder(SimWorld world) {
            this.world = world;
            this.solids = world.createBody(BodySpec.fixed(Pose3.IDENTITY).withName("rebuilt field"));
            bodies.add(solids);
        }

        void box(Alliance a, double x0, double y0, double z0, double x1, double y1, double z1, ShapeOptions o) {
            world.addShape(solids, boxSpec(a, x0, y0, z0, x1, y1, z1), o);
        }

        ShapeHandle sensor(Alliance a, double x0, double y0, double z0, double x1, double y1, double z1) {
            BodyHandle body = world.createBody(BodySpec.fixed(Pose3.IDENTITY));
            bodies.add(body);
            return world.addShape(body, boxSpec(a, x0, y0, z0, x1, y1, z1), ShapeOptions.sensor(CollisionLayers.SENSOR));
        }

        /** Triangular prism with its ridge along y at ridgeX. */
        void prism(Alliance a, double x0, double x1, double ridgeX, double height, double y0, double y1,
                ShapeOptions o) {
            List<Vec3> points = new ArrayList<>();
            for (Vec3 p : List.of(new Vec3(x0, y0, 0), new Vec3(x1, y0, 0), new Vec3(x0, y1, 0), new Vec3(x1, y1, 0),
                    new Vec3(ridgeX, y0, height), new Vec3(ridgeX, y1, height))) {
                points.add(a == Alliance.BLUE ? p : mirror(p));
            }
            world.addShape(solids, new ShapeSpec.Hull(points, Pose3.IDENTITY), o);
        }

        void capsule(Alliance a, Vec3 p1, Vec3 p2, double radius, ShapeOptions o) {
            Vec3 q1 = a == Alliance.BLUE ? p1 : mirror(p1);
            Vec3 q2 = a == Alliance.BLUE ? p2 : mirror(p2);
            world.addShape(solids, new ShapeSpec.Capsule(q1, q2, radius), o);
        }

        private static ShapeSpec.Box boxSpec(Alliance a, double x0, double y0, double z0, double x1, double y1,
                double z1) {
            if (a == Alliance.RED) {
                double mx0 = LENGTH - x1;
                double mx1 = LENGTH - x0;
                double my0 = WIDTH - y1;
                double my1 = WIDTH - y0;
                x0 = mx0;
                x1 = mx1;
                y0 = my0;
                y1 = my1;
            }
            Vec3 half = new Vec3((x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2);
            Vec3 center = new Vec3((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
            return new ShapeSpec.Box(half, Pose3.ofXYYaw(center.x(), center.y(), center.z(), 0));
        }
    }
}
