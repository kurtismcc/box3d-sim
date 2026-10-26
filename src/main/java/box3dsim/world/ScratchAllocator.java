package box3dsim.world;

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;

/** Bump allocator over one fixed segment, reset at the start of each public world call. */
final class ScratchAllocator implements SegmentAllocator {
    private final Arena overflow;
    private final MemorySegment segment;
    private long offset;

    ScratchAllocator(Arena arena, long capacity) {
        this.overflow = arena;
        this.segment = arena.allocate(capacity, 16);
    }

    void reset() {
        offset = 0;
    }

    @Override
    public MemorySegment allocate(long byteSize, long byteAlignment) {
        long start = (offset + byteAlignment - 1) & -byteAlignment;
        if (start + byteSize > segment.byteSize()) {
            // Rare oversized request; served from the world arena and freed when the world closes.
            return overflow.allocate(byteSize, byteAlignment);
        }
        offset = start + byteSize;
        return segment.asSlice(start, byteSize).fill((byte) 0);
    }
}
