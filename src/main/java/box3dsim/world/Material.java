package box3dsim.world;

/** Surface material. Rolling resistance applies to spheres and capsules only. */
public record Material(double friction, double restitution, double rollingResistance) {
    public static final Material DEFAULT = new Material(0.6, 0.0, 0.0);
}
