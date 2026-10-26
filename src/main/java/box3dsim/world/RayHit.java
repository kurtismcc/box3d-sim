package box3dsim.world;

import box3dsim.math.Vec3;

/** Closest ray cast hit. */
public record RayHit(ShapeHandle shape, Vec3 point, Vec3 normal, double fraction) {}
