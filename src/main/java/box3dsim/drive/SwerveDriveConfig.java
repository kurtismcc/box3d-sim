package box3dsim.drive;

import java.util.List;

/** Swerve chassis: modules, mass properties, bumper and superstructure geometry, tire and gyro models. */
public record SwerveDriveConfig(
        List<SwerveModuleConfig> modules,
        double mass,
        double yawMoi,
        double cgHeight,
        double bumperLengthX,
        double bumperWidthY,
        double bumperBottom,
        double bumperHeight,
        double frameInset,
        double robotHeight,
        TireConfig tire,
        GyroConfig gyro,
        boolean loadTransfer) {

    public SwerveDriveConfig {
        modules = List.copyOf(modules);
        if (modules.size() < 2) throw new IllegalArgumentException("need at least 2 modules");
        if (mass <= 10 || mass >= 80) throw new IllegalArgumentException("mass out of range (10, 80) kg: " + mass);
        if (yawMoi <= 0 || bumperLengthX <= 0 || bumperWidthY <= 0 || bumperHeight <= 0 || robotHeight <= 0) {
            throw new IllegalArgumentException("sizes and inertia must be positive");
        }
        if (robotHeight <= bumperBottom + bumperHeight) {
            throw new IllegalArgumentException("robotHeight must be above the bumper top");
        }
    }
}
