package box3dsim.gamepiece;

import box3dsim.world.Material;

/**
 * Spherical game piece.
 *
 * @param dragCoefficient C_d
 * @param liftSlope C_L = min(liftMax, liftSlope · S), with spin ratio S = r|ω|/|v|
 * @param spinDecayPerSecond exponential decay rate of |ω| while airborne
 */
public record GamePieceType(
        String name,
        double radius,
        double mass,
        Material material,
        double dragCoefficient,
        double liftSlope,
        double liftMax,
        double spinDecayPerSecond,
        double linearDamping,
        double angularDamping) {

    public double density() {
        return mass / (4.0 / 3.0 * Math.PI * radius * radius * radius);
    }
}
