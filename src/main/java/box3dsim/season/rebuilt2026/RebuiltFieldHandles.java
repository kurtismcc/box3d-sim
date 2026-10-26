package box3dsim.season.rebuilt2026;

import box3dsim.arena.Alliance;
import box3dsim.world.BodyHandle;
import box3dsim.world.ShapeHandle;
import java.util.List;
import java.util.Map;

/** Sensor shapes and static bodies created by RebuiltField.build. */
public record RebuiltFieldHandles(
        Map<Alliance, ShapeHandle> hubSensors, Map<Alliance, ShapeHandle> corralSensors, List<BodyHandle> staticBodies) {}
