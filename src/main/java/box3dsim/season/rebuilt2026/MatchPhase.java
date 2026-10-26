package box3dsim.season.rebuilt2026;

/** REBUILT match periods (game manual 6.4); FREE_PLAY and POST_MATCH are outside a timed match. */
public enum MatchPhase {
    FREE_PLAY,
    AUTO,
    AUTO_TO_TELEOP,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    END_GAME,
    POST_MATCH
}
