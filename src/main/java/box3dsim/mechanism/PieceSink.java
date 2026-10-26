package box3dsim.mechanism;

import box3dsim.gamepiece.GamePieceType;

/** Destination for pieces an intake acquires. */
public interface PieceSink {
    boolean canAccept();

    void accept(GamePieceType type);
}
