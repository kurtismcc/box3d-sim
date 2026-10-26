package box3dsim.drive;

/**
 * Slip-velocity tire curve: force = μ(σ)·N·tanh(σ / slipVelocity), μ(σ) = μk + (μs − μk)·exp(−σ / slipDecayVelocity).
 */
public record TireConfig(double staticMu, double kineticMu, double slipVelocity, double slipDecayVelocity) {
    public static TireConfig defaults() {
        return new TireConfig(1.2, 1.0, 0.08, 0.5);
    }
}
