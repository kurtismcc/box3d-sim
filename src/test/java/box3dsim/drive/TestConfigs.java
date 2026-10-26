package box3dsim.drive;

import box3dsim.motor.DcMotor;
import java.util.ArrayList;
import java.util.List;

/** Swerve configs for tests; the 2026-Offseason robot's own preset lives in the robot repo. */
public final class TestConfigs {
    private TestConfigs() {}

    /** MAXSwerve-style chassis with the 2026-Offseason robot's numbers. */
    public static SwerveDriveConfig maxSwerve() {
        return maxSwerve(GyroConfig.ideal(), true);
    }

    public static SwerveDriveConfig maxSwerve(GyroConfig gyro, boolean loadTransfer) {
        double d = 0.3493;
        double[][] positions = {{d, d}, {d, -d}, {-d, d}, {-d, -d}};
        List<SwerveModuleConfig> modules = new ArrayList<>();
        for (double[] p : positions) {
            modules.add(new SwerveModuleConfig(p[0], p[1], DcMotor.neoVortex(1), 3.625, 0.0381, 0.002, 0.15,
                    DcMotor.neo550(1), 9424.0 / 203.0, 0.004, 0.1));
        }
        return new SwerveDriveConfig(modules, 50, 6.883, 0.20, 0.9, 0.9, 0.0635, 0.0825, 0.08, 0.55,
                TireConfig.defaults(), gyro, loadTransfer);
    }
}
