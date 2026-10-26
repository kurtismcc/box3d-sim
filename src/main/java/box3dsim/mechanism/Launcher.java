package box3dsim.mechanism;

import box3dsim.gamepiece.GamePieceType;

/** Takes a staged piece; returns false when it can't launch now (the piece stays staged). */
public interface Launcher {
    boolean launch(GamePieceType type);
}
