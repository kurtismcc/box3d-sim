package box3dsim.drive;

/**
 * Wheel-ground contact for a chassis free in all six degrees of freedom. Each wheel is a disc in its steering plane,
 * sampled with rays against FIELD geometry; a spring-damper on its penetration gives the normal load. Swerve modules
 * have no suspension, so the spring stands for tread, tire and frame compliance and the wheel never droops below its
 * rest position.
 *
 * @param stiffness N/m per module
 * @param damping N·s/m per module
 * @param groundClearance m, underside of the frame (the skirt under the bumpers) above the carpet
 * @param rollMoi kg·m² about the robot x axis through the cg
 * @param pitchMoi kg·m² about the robot y axis through the cg
 */
public record SuspensionConfig(
        double stiffness, double damping, double groundClearance, double rollMoi, double pitchMoi) {

    public SuspensionConfig {
        if (stiffness <= 0 || damping < 0 || groundClearance <= 0 || rollMoi <= 0 || pitchMoi <= 0) {
            throw new IllegalArgumentException("suspension values must be positive");
        }
    }
}
