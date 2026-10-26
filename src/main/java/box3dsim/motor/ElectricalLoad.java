package box3dsim.motor;

/** Something that draws current from the battery. */
public interface ElectricalLoad {
    /** Supply (battery-side) current in amps for the step just simulated. Negative when regenerating. */
    double supplyCurrent();

    /** Battery-side current through a motor controller, by conservation of power. */
    static double supplyFromStator(double statorCurrent, double appliedVolts, double batteryVolts) {
        return batteryVolts < 1e-6 ? 0 : statorCurrent * appliedVolts / batteryVolts;
    }
}
