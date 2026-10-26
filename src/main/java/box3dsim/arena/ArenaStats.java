package box3dsim.arena;

/** Timing of the last Arena.step(). */
public record ArenaStats(long physicsNanos, long totalNanos, int awakePieces) {}
