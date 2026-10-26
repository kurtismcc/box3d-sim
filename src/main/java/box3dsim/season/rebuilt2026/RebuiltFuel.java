package box3dsim.season.rebuilt2026;

import box3dsim.gamepiece.GamePieceType;

/** The 2026 FUEL game piece: 15 cm foam ball, 0.203–0.227 kg (game manual 5.10.1). */
public final class RebuiltFuel {
    /** Drag, lift and spin decay are ASSUMPTIONS (typical foam sphere values). */
    public static final GamePieceType TYPE =
            new GamePieceType("Fuel", 0.075, 0.215, RebuiltMaterials.FUEL, 0.50, 0.6, 0.30, 0.5, 0.0, 0.0);

    private RebuiltFuel() {}
}
