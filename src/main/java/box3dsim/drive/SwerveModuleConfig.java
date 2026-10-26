package box3dsim.drive;

import box3dsim.motor.DcMotor;

/**
 * One swerve module. Position is in the robot frame (+x forward, +y left).
 *
 * @param wheelMoi kg·m² at the wheel axle, including the reflected drive rotor and gears
 * @param driveFrictionTorque Coulomb friction, N·m at the wheel
 * @param steerMoi kg·m² about the steer axis
 * @param steerFrictionTorque Coulomb friction, N·m at the steer axis
 */
public record SwerveModuleConfig(
        double x,
        double y,
        DcMotor driveMotor,
        double driveGearing,
        double wheelRadius,
        double wheelMoi,
        double driveFrictionTorque,
        DcMotor steerMotor,
        double steerGearing,
        double steerMoi,
        double steerFrictionTorque) {}
