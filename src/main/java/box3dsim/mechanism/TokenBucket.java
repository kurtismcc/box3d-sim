package box3dsim.mechanism;

/** Rate limiter: refills toward one token; a transfer spends one. */
final class TokenBucket {
    private double tokens;

    void refill(double rate, double dt) {
        if (rate > 0) {
            tokens = Math.min(1, tokens + rate * dt);
        }
    }

    boolean ready() {
        return tokens >= 1;
    }

    void spend() {
        tokens -= 1;
    }

    void reset() {
        tokens = 0;
    }
}
