package box3dsim.world;

/**
 * Per-shape options. Shapes added with updateBodyMass false keep a mass override set with
 * SimWorld.setMassProperties; a shape that updates body mass discards it.
 */
public record ShapeOptions(
        Material material,
        double density,
        CollisionFilter filter,
        boolean isSensor,
        boolean enableSensorEvents,
        boolean updateBodyMass) {

    /** Solid shape. Game pieces get sensor events so sensors can see them. */
    public static ShapeOptions solid(Material material, double density, long layerBit) {
        return new ShapeOptions(
                material, density, CollisionLayers.of(layerBit), false, layerBit == CollisionLayers.GAME_PIECE, true);
    }

    public static ShapeOptions sensor(long layerBit) {
        return new ShapeOptions(Material.DEFAULT, 0, CollisionLayers.of(layerBit), true, true, false);
    }

    public ShapeOptions withSensorEvents(boolean enable) {
        return new ShapeOptions(material, density, filter, isSensor, enable, updateBodyMass);
    }

    public ShapeOptions withUpdateBodyMass(boolean update) {
        return new ShapeOptions(material, density, filter, isSensor, enableSensorEvents, update);
    }
}
