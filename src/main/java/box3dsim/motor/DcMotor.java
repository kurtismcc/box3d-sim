package box3dsim.motor;

/** Brushed/brushless DC motor constants; values for motorCount motors geared together. */
public record DcMotor(
        String name,
        double nominalVoltage,
        double stallTorque,
        double stallCurrent,
        double freeCurrent,
        double freeSpeedRadPerSec,
        int motorCount) {

    private static final double RPM_TO_RAD_PER_SEC = 2 * Math.PI / 60;

    /** Winding resistance, ohms. */
    public double resistance() {
        return nominalVoltage / stallCurrent;
    }

    /** Speed constant, rad/s per volt. */
    public double kv() {
        return freeSpeedRadPerSec / (nominalVoltage - resistance() * freeCurrent);
    }

    /** Torque constant, N·m per amp. */
    public double kt() {
        return stallTorque / stallCurrent;
    }

    /** Stator current at rotor speed and applied voltage. */
    public double current(double rotorSpeedRadPerSec, double volts) {
        return (volts - rotorSpeedRadPerSec / kv()) / resistance();
    }

    public double torque(double currentAmps) {
        return kt() * currentAmps;
    }

    public double voltage(double torque, double rotorSpeedRadPerSec) {
        return torque / kt() * resistance() + rotorSpeedRadPerSec / kv();
    }

    private static DcMotor of(String name, double stallTorque, double stallCurrent, double freeCurrent,
            double freeRpm, int count) {
        return new DcMotor(name, 12, stallTorque * count, stallCurrent * count, freeCurrent * count,
                freeRpm * RPM_TO_RAD_PER_SEC, count);
    }

    public static DcMotor neoVortex(int count) {
        return of("NEO Vortex", 3.60, 211, 3.6, 6784, count);
    }

    public static DcMotor neo(int count) {
        return of("NEO", 2.6, 105, 1.8, 5676, count);
    }

    public static DcMotor neo550(int count) {
        return of("NEO 550", 0.97, 100, 1.4, 11000, count);
    }

    public static DcMotor krakenX60(int count) {
        return of("Kraken X60", 7.09, 366, 2, 6000, count);
    }
}
