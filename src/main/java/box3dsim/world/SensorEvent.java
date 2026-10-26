package box3dsim.world;

/** A visitor shape starting or stopping overlap with a sensor shape. */
public record SensorEvent(ShapeHandle sensor, ShapeHandle visitor) {}
