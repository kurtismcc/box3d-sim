package box3dsim.mechanism;

import box3dsim.gamepiece.GamePieceType;
import box3dsim.math.Vec3;
import box3dsim.motor.LumpedMechanism;

/**
 * Ground intake.
 *
 * @param pivot deploy mechanism; positions in mechanism rad
 * @param deployedFraction the intake works once deploy progress reaches this (0..1)
 * @param zoneCenter sensor box center in the robot frame, where the deployed roller sweeps
 */
public record IntakeConfig(
        RollerConfig roller,
        LumpedMechanism.Config pivot,
        double deployedPosition,
        double stowedPosition,
        double deployedFraction,
        Vec3 zoneCenter,
        Vec3 zoneHalfExtents,
        GamePieceType pieceType) {}
