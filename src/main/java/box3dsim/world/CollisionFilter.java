package box3dsim.world;

/** Category and mask bits; two shapes collide when each one's category is in the other's mask. */
public record CollisionFilter(long categoryBits, long maskBits) {}
