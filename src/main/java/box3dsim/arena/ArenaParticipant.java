package box3dsim.arena;

/** Something stepped by the Arena around each world step. */
public interface ArenaParticipant {
    /** Before the world step: read inputs, apply forces. */
    default void prePhysics(Arena arena, double dt) {}

    /** After the world step: read results, handle sensor events, spawn or remove bodies. */
    default void postPhysics(Arena arena, double dt) {}

    /** Field reset (new match). */
    default void reset(Arena arena) {}
}
