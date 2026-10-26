package box3dsim.mechanism;

import box3dsim.math.Pose3;
import box3dsim.math.Vec3;

/** Notified for every piece a shooter launches. */
public interface LaunchListener {
    void onLaunch(Pose3 robotPose, Vec3 exitVelocity);
}
