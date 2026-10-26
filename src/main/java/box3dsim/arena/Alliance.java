package box3dsim.arena;

/** FRC alliance color. */
public enum Alliance {
    BLUE,
    RED;

    public Alliance opponent() {
        return this == BLUE ? RED : BLUE;
    }
}
