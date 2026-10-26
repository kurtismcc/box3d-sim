package box3dsim.motor;

/** One rigid degree of freedom: motor(s) through a reduction into a load inertia. */
public final class LumpedMechanism implements ElectricalLoad {
    /**
     * @param gearing rotor rotations per mechanism rotation
     * @param moi kg·m² at the mechanism, including the reflected rotor
     * @param frictionTorque Coulomb friction, N·m at the mechanism
     * @param viscousDamping N·m·s/rad at the mechanism
     * @param minPosition mechanism rad; NEGATIVE_INFINITY for none
     * @param maxPosition mechanism rad; POSITIVE_INFINITY for none
     * @param statorCurrentLimit amps; POSITIVE_INFINITY for none
     */
    public record Config(
            DcMotor motor,
            double gearing,
            double moi,
            double frictionTorque,
            double viscousDamping,
            double minPosition,
            double maxPosition,
            double statorCurrentLimit) {

        public static Config of(DcMotor motor, double gearing, double moi) {
            return new Config(motor, gearing, moi, 0, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY);
        }

        public Config withFriction(double coulomb, double viscous) {
            return new Config(motor, gearing, moi, coulomb, viscous, minPosition, maxPosition, statorCurrentLimit);
        }

        public Config withLimits(double min, double max) {
            return new Config(motor, gearing, moi, frictionTorque, viscousDamping, min, max, statorCurrentLimit);
        }

        public Config withCurrentLimit(double amps) {
            return new Config(motor, gearing, moi, frictionTorque, viscousDamping, minPosition, maxPosition, amps);
        }
    }

    private final Config config;
    private final Battery battery;
    private double requestedVolts;
    private boolean coastAtZero;
    private double externalTorque;
    private double position;
    private double velocity;
    private double appliedVolts;
    private double statorCurrent;
    private double supplyCurrent;

    public LumpedMechanism(Config config, Battery battery) {
        this.config = config;
        this.battery = battery;
    }

    /** Controller output for the next step; clamped to the battery voltage. */
    public void setInput(double volts, boolean coastAtZero) {
        this.requestedVolts = volts;
        this.coastAtZero = coastAtZero;
    }

    /** Extra torque at the mechanism for the next step only. Accumulates. */
    public void addExternalTorque(double newtonMeters) {
        externalTorque += newtonMeters;
    }

    /** Removes kinetic energy instantly, keeping the direction of motion. */
    public void removeEnergy(double joules) {
        double w2 = velocity * velocity - 2 * joules / config.moi;
        velocity = Math.copySign(Math.sqrt(Math.max(0, w2)), velocity);
    }

    public void step(double dt) {
        DcMotor m = config.motor;
        double g = config.gearing;
        double j = config.moi;
        double r = m.resistance();
        double kv = m.kv();
        double kt = m.kt();
        double vBat = battery.voltage();

        double v = Math.clamp(requestedVolts, -vBat, vBat);
        boolean open = coastAtZero && Math.abs(v) < 1e-9;
        double a = 0;
        double b = 0;
        if (!open) {
            double current = (v - g * velocity / kv) / r;
            if (Math.abs(current) > config.statorCurrentLimit) {
                v = Math.copySign(config.statorCurrentLimit, current) * r + g * velocity / kv;
                v = Math.clamp(v, -vBat, vBat);
            }
            a = g * kt / r;
            b = g * g * kt / (kv * r);
        }

        // Back-EMF and viscous damping are integrated implicitly so tiny inertias stay stable.
        double w = (velocity + dt / j * (a * v + externalTorque)) / (1 + dt * (b + config.viscousDamping) / j);

        double delta = dt * config.frictionTorque / j;
        w = Math.abs(w) <= delta ? 0 : w - Math.copySign(delta, w);

        double p = position + w * dt;
        if (p < config.minPosition) {
            p = config.minPosition;
            w = Math.max(w, 0);
        } else if (p > config.maxPosition) {
            p = config.maxPosition;
            w = Math.min(w, 0);
        }

        position = p;
        velocity = w;
        appliedVolts = open ? 0 : v;
        statorCurrent = open ? 0 : (v - g * w / kv) / r;
        supplyCurrent = ElectricalLoad.supplyFromStator(statorCurrent, appliedVolts, vBat);
        externalTorque = 0;
    }

    /** Mechanism rad. */
    public double position() {
        return position;
    }

    /** Mechanism rad/s. */
    public double velocity() {
        return velocity;
    }

    public double rotorPosition() {
        return position * config.gearing;
    }

    public double rotorVelocity() {
        return velocity * config.gearing;
    }

    public double appliedVolts() {
        return appliedVolts;
    }

    public double statorCurrent() {
        return statorCurrent;
    }

    @Override
    public double supplyCurrent() {
        return supplyCurrent;
    }

    public void setState(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    public Config config() {
        return config;
    }
}
