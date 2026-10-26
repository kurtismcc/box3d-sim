package box3dsim.season.rebuilt2026;

import box3dsim.world.Material;

/** Surface materials for the 2026 REBUILT field. Box3D mixes restitution as max and friction as geometric mean. */
public final class RebuiltMaterials {
    /** Rolling resistance stays 0 here; it would mix into Box3D's contact (max) and stop sliding FUEL dead. */
    public static final Material CARPET = new Material(0.8, 0.0, 0.0);
    public static final Material POLYCARB = new Material(0.4, 0.3, 0.0);
    public static final Material HDPE = new Material(0.3, 0.2, 0.0);
    /**
     * ASSUMPTION: foam ball bounce and roll; calibrate against the real FUEL. Rolling resistance is the C_rr that
     * GamePieces applies on the floor (0.05: a FUEL rolled at 2 m/s stops after about 4 m).
     */
    public static final Material FUEL = new Material(0.6, 0.35, 0.05);

    private RebuiltMaterials() {}
}
