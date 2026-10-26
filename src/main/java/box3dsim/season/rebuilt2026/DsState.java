package box3dsim.season.rebuilt2026;

/** Driver station state as the robot sees it, sampled once per robot loop. */
public record DsState(boolean enabled, boolean autonomous, boolean teleop) {
    public static final DsState DISABLED = new DsState(false, false, false);
}
