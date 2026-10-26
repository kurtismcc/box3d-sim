package box3dsim.mechanism;

import box3dsim.motor.LumpedMechanism;

/**
 * A motor-driven roller. directionSign maps the robot's voltage convention so that positive surface speed always
 * moves pieces inward or toward the shooter.
 *
 * @param efficiency fraction of surface speed that becomes piece throughput
 * @param minSurfaceSpeed m/s below which the roller moves no pieces
 */
public record RollerConfig(
        LumpedMechanism.Config mechanism,
        double rollerRadius,
        double directionSign,
        double efficiency,
        double minSurfaceSpeed) {

    /** Surface speed in the feed direction, m/s. */
    double surfaceSpeed(LumpedMechanism roller) {
        return roller.velocity() * rollerRadius * directionSign;
    }
}
