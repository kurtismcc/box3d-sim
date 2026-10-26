package box3dsim.mechanism;

import box3dsim.gamepiece.GamePieceType;
import box3dsim.math.Vec3;
import box3dsim.motor.LumpedMechanism;
import java.util.function.DoubleUnaryOperator;

/**
 * Flywheel and hood shooter.
 *
 * @param shotDirectionSign sign of flywheel velocity that shoots
 * @param exitSpeedRatio v_exit = ratio · |ω|·r (about 0.5 for a single wheel against a hood)
 * @param spinRatio backspin |ω_ball| = spinRatio · v_exit / r_ball
 * @param energyEfficiency the flywheel loses E_ball / efficiency per shot
 * @param hoodPositionToPitch hood mechanism rad → launch pitch above horizontal, rad
 * @param exitPointRobot launch point in the robot frame; must be outside the robot's collision boxes
 * @param exitYawRobot launch direction yaw in the robot frame (0 = robot +x)
 */
public record ShooterConfig(
        LumpedMechanism.Config flywheel,
        double flywheelRadius,
        double shotDirectionSign,
        double exitSpeedRatio,
        double spinRatio,
        double energyEfficiency,
        double minExitSpeed,
        LumpedMechanism.Config hood,
        DoubleUnaryOperator hoodPositionToPitch,
        Vec3 exitPointRobot,
        double exitYawRobot,
        GamePieceType pieceType) {}
