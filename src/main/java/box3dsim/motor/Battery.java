package box3dsim.motor;

/**
 * Battery with internal resistance. Load current is low-pass filtered: motor currents depend on battery voltage
 * and vice versa, and without the filter that one-step-delayed loop oscillates once enough motors are loaded.
 */
public final class Battery {
    private final double openCircuitVolts;
    private final double internalResistance;
    private final double brownoutVolts;
    private final double filterTimeConstant;
    private double filteredCurrent;
    private double voltage;
    private boolean brownedOut;

    public Battery(double openCircuitVolts, double internalResistanceOhms, double brownoutVolts,
            double filterTimeConstant) {
        this.openCircuitVolts = openCircuitVolts;
        this.internalResistance = internalResistanceOhms;
        this.brownoutVolts = brownoutVolts;
        this.filterTimeConstant = filterTimeConstant;
        reset();
    }

    /** 12.6 V, 0.020 Ω, 6.8 V brownout, 0.04 s current filter. */
    public static Battery defaults() {
        return new Battery(12.6, 0.020, 6.8, 0.04);
    }

    /** Call once per physics step with the total supply current drawn during that step. */
    public void update(double totalSupplyCurrentAmps, double dt) {
        double alpha = dt / (filterTimeConstant + dt);
        filteredCurrent += alpha * (totalSupplyCurrentAmps - filteredCurrent);
        double loaded = openCircuitVolts - internalResistance * filteredCurrent;
        brownedOut = loaded < brownoutVolts;
        voltage = Math.max(brownoutVolts, loaded);
    }

    /** Loaded terminal voltage, never below the brownout voltage. */
    public double voltage() {
        return voltage;
    }

    public double filteredCurrent() {
        return filteredCurrent;
    }

    public boolean isBrownedOut() {
        return brownedOut;
    }

    public void reset() {
        filteredCurrent = 0;
        voltage = openCircuitVolts;
        brownedOut = false;
    }
}
