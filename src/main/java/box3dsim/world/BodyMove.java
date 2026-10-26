package box3dsim.world;

import box3dsim.math.Pose3;

/** A body that moved during the last step. */
public record BodyMove(BodyHandle body, Pose3 pose, boolean fellAsleep) {}
