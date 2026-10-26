package box3dsim.season.rebuilt2026;

import box3dsim.arena.Alliance;
import box3dsim.math.Vec3;
import java.util.ArrayList;
import java.util.List;

/** Starting FUEL positions, matching AdvantageScope's 2026 field model; red mirrors blue. */
public final class RebuiltFuelLayout {
    private static final double PITCH = 0.1524;
    private static final double Z = 0.075;

    private RebuiltFuelLayout() {}

    /** 360 FUEL: 12 columns × 30 rows at 6 in pitch, with the 2 in center-divider gap along y. */
    public static List<Vec3> neutralZone() {
        List<Vec3> out = new ArrayList<>(360);
        for (int i = 0; i < 12; i++) {
            double x = RebuiltField.LENGTH / 2 + (i - 5.5) * PITCH;
            for (int k = 0; k < 15; k++) {
                double offset = 0.1016 + k * PITCH;
                out.add(new Vec3(x, RebuiltField.WIDTH / 2 - offset, Z));
                out.add(new Vec3(x, RebuiltField.WIDTH / 2 + offset, Z));
            }
        }
        return out;
    }

    /** 24 FUEL in a 4 × 6 grid against the alliance wall. */
    public static List<Vec3> depot(Alliance a) {
        List<Vec3> out = new ArrayList<>(24);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 6; j++) {
                out.add(side(a, new Vec3(0.0762 + i * PITCH, 5.5839 + j * PITCH, Z)));
            }
        }
        return out;
    }

    /** 24 display slots in the CHUTE, back to front; show the first n stored. */
    public static List<Vec3> chuteDisplay(Alliance a) {
        List<Vec3> out = new ArrayList<>(24);
        for (int j = 0; j < 4; j++) {
            out.add(side(a, new Vec3(-0.6389, 0.4372 + j * PITCH, 0.9616)));
        }
        double[] xs = {-0.5114, -0.3642, -0.2170, -0.0698};
        double[] zs = {0.9274, 0.8880, 0.8485, 0.8091};
        for (int row = 0; row < 4; row++) {
            for (int j = 0; j < 5; j++) {
                out.add(side(a, new Vec3(xs[row], 0.3610 + j * PITCH, zs[row])));
            }
        }
        return out;
    }

    /** 30 display slots on the CORRAL floor; show the first n stored. ASSUMPTION (visual only). */
    public static List<Vec3> corralDisplay(Alliance a) {
        List<Vec3> out = new ArrayList<>(30);
        for (int c = 0; c < 5; c++) {
            for (int r = 0; r < 6; r++) {
                out.add(side(a, new Vec3(-0.85 + c * 0.16, 0.30 + r * 0.14, Z)));
            }
        }
        return out;
    }

    private static Vec3 side(Alliance a, Vec3 blue) {
        return a == Alliance.BLUE ? blue : RebuiltField.mirror(blue);
    }
}
