package box3dsim.mechanism;

import box3dsim.gamepiece.GamePieceType;

/** Hopper with an indexer into one staged slot and a feeder into the launcher. */
public record StorageConfig(
        int capacity, int preload, RollerConfig indexer, RollerConfig feeder, GamePieceType pieceType) {}
