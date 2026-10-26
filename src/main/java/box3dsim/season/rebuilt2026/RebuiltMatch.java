package box3dsim.season.rebuilt2026;

import box3dsim.arena.Alliance;
import box3dsim.arena.Arena;
import box3dsim.arena.ArenaParticipant;
import box3dsim.gamepiece.GamePiece;
import box3dsim.gamepiece.GamePieces;
import box3dsim.math.Pose3;
import box3dsim.math.Quat;
import box3dsim.math.Vec3;
import box3dsim.world.SensorEvent;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

/**
 * REBUILT match rules: the sim's own match clock (the 2027 sim GUI runs no timed match), HUB shifts and scoring, HUB
 * exits, OUTPOST human-player actions and the G407 foul. Rules and sources: docs/steps/09-rebuilt-rules.md.
 */
public final class RebuiltMatch implements ArenaParticipant {
    public record Config(long seed, boolean spawnNeutralZone, boolean spawnDepots, int robotPreload) {
        public static Config defaults() {
            return new Config(4470, true, true, 8);
        }
    }

    private static final double AUTO_LENGTH = 20;
    private static final double TELEOP_LENGTH = 140;
    private static final double SCORING_GRACE = 3;
    private static final double[] SHIFT_STARTS = {10, 35, 60, 85, 110};
    private static final int CHUTE_CAPACITY = 25;
    /** ASSUMPTION: CORRAL floor holds about 30 FUEL. */
    private static final int CORRAL_CAPACITY = 30;
    private static final int OUTPOST_START = 24;
    private static final double CHUTE_RELEASE_INTERVAL = 0.15;
    private static final int FOUL_MAJOR = 15;

    // ASSUMPTION: HUB processing delay (FIRST measured a 1.32 s mean, all under 3 s) and exit geometry/speeds.
    private static final double COUNT_DELAY_MIN = 0.8;
    private static final double COUNT_DELAY_MAX = 1.8;
    private static final double[] EXIT_Y_OFFSETS = {-0.4477, -0.1492, 0.1492, 0.4477};
    private static final double[] EXIT_YAWS_DEG = {-33.75, -11.25, 11.25, 33.75};

    private final Arena arena;
    private final RebuiltFieldHandles field;
    private final Config config;
    private final Random random;

    private DsState ds = DsState.DISABLED;
    private boolean prevEnabledAuto;
    private boolean prevEnabledTeleop;
    private boolean teleopArmed;
    private MatchPhase phase = MatchPhase.FREE_PLAY;
    private double autoStart;
    private double teleopStart;
    private Optional<Alliance> firstInactive = Optional.empty();

    private final Map<Alliance, Integer> score = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Integer> autoFuel = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Integer> fuelProcessed = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Integer> freePlayFuel = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Integer> fouls = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Integer> chute = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Integer> corral = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Boolean> releasing = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Double> nextRelease = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Double> deactivatedAt = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Boolean> prevActive = new EnumMap<>(Alliance.class);
    private final List<PendingFuel> pending = new ArrayList<>();
    private String lastFoul = "";

    private record PendingFuel(Alliance alliance, double countTime) {}

    public RebuiltMatch(Arena arena, RebuiltFieldHandles field, Config config) {
        this.arena = arena;
        this.field = field;
        this.config = config;
        this.random = new Random(config.seed());
        arena.gamePieces().setFieldBounds(0, RebuiltField.LENGTH, 0, RebuiltField.WIDTH);
        resetCounters();
    }

    public void setDsState(DsState ds) {
        this.ds = ds;
    }

    /** Back to FREE_PLAY with fresh field FUEL, scores and OUTPOSTs. */
    public void resetMatch() {
        GamePieces pieces = arena.gamePieces();
        pieces.clear();
        pending.clear();
        resetCounters();
        if (config.spawnNeutralZone()) {
            for (Vec3 p : RebuiltFuelLayout.neutralZone()) {
                pieces.spawn(RebuiltFuel.TYPE, p, Vec3.ZERO, Vec3.ZERO, false);
            }
        }
        if (config.spawnDepots()) {
            for (Alliance a : Alliance.values()) {
                for (Vec3 p : RebuiltFuelLayout.depot(a)) {
                    pieces.spawn(RebuiltFuel.TYPE, p, Vec3.ZERO, Vec3.ZERO, false);
                }
            }
        }
    }

    private void resetCounters() {
        phase = MatchPhase.FREE_PLAY;
        firstInactive = Optional.empty();
        prevEnabledAuto = false;
        prevEnabledTeleop = false;
        teleopArmed = false;
        lastFoul = "";
        for (Alliance a : Alliance.values()) {
            score.put(a, 0);
            autoFuel.put(a, 0);
            fuelProcessed.put(a, 0);
            freePlayFuel.put(a, 0);
            fouls.put(a, 0);
            chute.put(a, OUTPOST_START);
            corral.put(a, 0);
            releasing.put(a, false);
            nextRelease.put(a, 0.0);
            deactivatedAt.put(a, Double.NEGATIVE_INFINITY);
            prevActive.put(a, true);
        }
    }

    @Override
    public void reset(Arena arena) {
        resetMatch();
    }

    @Override
    public void postPhysics(Arena arena, double dt) {
        double now = arena.time();
        updatePhase(now);
        for (Alliance a : Alliance.values()) {
            boolean active = hubActive(a);
            if (prevActive.get(a) && !active) {
                deactivatedAt.put(a, now);
            }
            prevActive.put(a, active);
        }
        handleSensorEvents(now);
        processPending(now);
        releaseChutes(now);
    }

    private void updatePhase(double now) {
        boolean enabledAuto = ds.enabled() && ds.autonomous();
        boolean enabledTeleop = ds.enabled() && ds.teleop();
        boolean autoRise = enabledAuto && !prevEnabledAuto;
        boolean teleopRise = enabledTeleop && !prevEnabledTeleop;
        prevEnabledAuto = enabledAuto;
        prevEnabledTeleop = enabledTeleop;

        switch (phase) {
            case FREE_PLAY -> {
                if (autoRise) {
                    phase = MatchPhase.AUTO;
                    autoStart = now;
                } else if (teleopRise) {
                    startTeleop(now);
                }
            }
            case AUTO -> {
                if (now - autoStart >= AUTO_LENGTH) {
                    phase = MatchPhase.AUTO_TO_TELEOP;
                    teleopArmed = false;
                }
            }
            case AUTO_TO_TELEOP -> {
                if (!ds.enabled() || ds.autonomous()) {
                    teleopArmed = true;
                }
                if (firstInactive.isEmpty() && now >= autoEnd() + SCORING_GRACE) {
                    pickFirstInactive();
                }
                if (teleopArmed && enabledTeleop) {
                    startTeleop(now);
                }
            }
            case POST_MATCH -> {}
            default -> {
                double t = now - teleopStart;
                if (t >= TELEOP_LENGTH) {
                    phase = MatchPhase.POST_MATCH;
                } else {
                    phase = teleopPhase(t);
                }
            }
        }
    }

    private void startTeleop(double now) {
        teleopStart = now;
        phase = MatchPhase.TRANSITION;
        if (firstInactive.isEmpty()) {
            pickFirstInactive();
        }
    }

    private void pickFirstInactive() {
        int blue = autoFuel.get(Alliance.BLUE);
        int red = autoFuel.get(Alliance.RED);
        if (blue == red) {
            firstInactive = Optional.of(random.nextBoolean() ? Alliance.BLUE : Alliance.RED);
        } else {
            firstInactive = Optional.of(blue > red ? Alliance.BLUE : Alliance.RED);
        }
    }

    private static MatchPhase teleopPhase(double t) {
        if (t < SHIFT_STARTS[0]) return MatchPhase.TRANSITION;
        if (t < SHIFT_STARTS[1]) return MatchPhase.SHIFT_1;
        if (t < SHIFT_STARTS[2]) return MatchPhase.SHIFT_2;
        if (t < SHIFT_STARTS[3]) return MatchPhase.SHIFT_3;
        if (t < SHIFT_STARTS[4]) return MatchPhase.SHIFT_4;
        return MatchPhase.END_GAME;
    }

    private double autoEnd() {
        return autoStart + AUTO_LENGTH;
    }

    private double teleopEnd() {
        return teleopStart + TELEOP_LENGTH;
    }

    private void handleSensorEvents(double now) {
        GamePieces pieces = arena.gamePieces();
        for (SensorEvent e : arena.world().sensorBeginEvents()) {
            Optional<GamePiece> piece = pieces.byShape(e.visitor());
            if (piece.isEmpty() || !piece.get().type().equals(RebuiltFuel.TYPE) || !piece.get().isAlive()) {
                continue;
            }
            for (Alliance a : Alliance.values()) {
                if (e.sensor().equals(field.hubSensors().get(a))) {
                    pieces.remove(piece.get());
                    double delay = COUNT_DELAY_MIN + random.nextDouble() * (COUNT_DELAY_MAX - COUNT_DELAY_MIN);
                    pending.add(new PendingFuel(a, now + delay));
                } else if (e.sensor().equals(field.corralSensors().get(a))) {
                    pieces.remove(piece.get());
                    corral.merge(a, 1, Integer::sum);
                    if (corral.get(a) > CORRAL_CAPACITY) {
                        // G427: excess FUEL must be entered onto the field immediately.
                        corral.merge(a, -1, Integer::sum);
                        spawnFromChute(a);
                    }
                }
            }
        }
    }

    private void processPending(double now) {
        Iterator<PendingFuel> it = pending.iterator();
        while (it.hasNext()) {
            PendingFuel f = it.next();
            if (now < f.countTime()) {
                continue;
            }
            it.remove();
            countFuel(f.alliance(), now);
            spawnFromHubExit(f.alliance());
        }
    }

    private void countFuel(Alliance a, double now) {
        fuelProcessed.merge(a, 1, Integer::sum);
        if (phase == MatchPhase.FREE_PLAY) {
            freePlayFuel.merge(a, 1, Integer::sum);
            return;
        }
        boolean inWindow = switch (phase) {
            case AUTO -> true;
            case AUTO_TO_TELEOP -> now - autoEnd() <= SCORING_GRACE;
            case POST_MATCH -> now - teleopEnd() <= SCORING_GRACE;
            default -> true;
        };
        boolean counts = hubActive(a) || now - deactivatedAt.get(a) <= SCORING_GRACE;
        if (inWindow && counts) {
            score.merge(a, 1, Integer::sum);
            if (phase == MatchPhase.AUTO || phase == MatchPhase.AUTO_TO_TELEOP) {
                autoFuel.merge(a, 1, Integer::sum);
            }
        }
    }

    private void spawnFromHubExit(Alliance a) {
        int exit = random.nextInt(EXIT_Y_OFFSETS.length);
        Vec3 position = new Vec3(5.2225 + 0.10, 4.0346 + EXIT_Y_OFFSETS[exit], 0.12);
        double speed = 1.0 + random.nextDouble() * 2.0;
        double yaw = Math.toRadians(EXIT_YAWS_DEG[exit] + (random.nextDouble() * 20 - 10));
        Vec3 velocity = new Vec3(speed * Math.cos(yaw), speed * Math.sin(yaw), 0);
        spawn(a, position, velocity);
    }

    private void releaseChutes(double now) {
        for (Alliance a : Alliance.values()) {
            if (!releasing.get(a)) {
                continue;
            }
            while (chute.get(a) > 0 && now >= nextRelease.get(a)) {
                chute.merge(a, -1, Integer::sum);
                spawnFromChute(a);
                nextRelease.put(a, nextRelease.get(a) + CHUTE_RELEASE_INTERVAL);
            }
            if (chute.get(a) == 0) {
                releasing.put(a, false);
            }
        }
    }

    private void spawnFromChute(Alliance a) {
        Vec3 position = new Vec3(0.10, 0.666 + (random.nextDouble() * 0.6 - 0.3), 0.80);
        Vec3 velocity = new Vec3(1.0 + random.nextDouble(), random.nextDouble() * 0.4 - 0.2, -0.3);
        spawn(a, position, velocity);
    }

    /** Spawns a FUEL given in blue coordinates, mirrored for red. */
    private void spawn(Alliance a, Vec3 bluePosition, Vec3 blueVelocity) {
        Vec3 p = a == Alliance.BLUE ? bluePosition : RebuiltField.mirror(bluePosition);
        Vec3 v = a == Alliance.BLUE ? blueVelocity : new Vec3(-blueVelocity.x(), -blueVelocity.y(), blueVelocity.z());
        arena.gamePieces().spawn(RebuiltFuel.TYPE, p, v, Vec3.ZERO, true);
    }

    public MatchPhase phase() {
        return phase;
    }

    /** Seconds left in AUTO or TELEOP; −1 outside a timed period. */
    public double matchTimeRemaining() {
        double now = arena.time();
        return switch (phase) {
            case AUTO -> Math.max(0, AUTO_LENGTH - (now - autoStart));
            case TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, END_GAME ->
                    Math.max(0, TELEOP_LENGTH - (now - teleopStart));
            default -> -1;
        };
    }

    /** Seconds left in the current phase; −1 outside a timed period. */
    public double phaseTimeRemaining() {
        double now = arena.time();
        double t = now - teleopStart;
        return switch (phase) {
            case AUTO -> Math.max(0, AUTO_LENGTH - (now - autoStart));
            case TRANSITION -> SHIFT_STARTS[0] - t;
            case SHIFT_1 -> SHIFT_STARTS[1] - t;
            case SHIFT_2 -> SHIFT_STARTS[2] - t;
            case SHIFT_3 -> SHIFT_STARTS[3] - t;
            case SHIFT_4 -> SHIFT_STARTS[4] - t;
            case END_GAME -> TELEOP_LENGTH - t;
            default -> -1;
        };
    }

    /** False between AUTO and TELEOP and after the match, when FMS would disable the robot. */
    public boolean outputsAllowed() {
        return phase != MatchPhase.AUTO_TO_TELEOP && phase != MatchPhase.POST_MATCH;
    }

    public boolean hubActive(Alliance a) {
        return switch (phase) {
            case SHIFT_1, SHIFT_3 -> firstInactive.map(f -> f != a).orElse(true);
            case SHIFT_2, SHIFT_4 -> firstInactive.map(f -> f == a).orElse(true);
            default -> true;
        };
    }

    public Optional<Alliance> firstInactive() {
        return firstInactive;
    }

    public int score(Alliance a) {
        return score.get(a);
    }

    public int autoFuel(Alliance a) {
        return autoFuel.get(a);
    }

    public int fuelProcessed(Alliance a) {
        return fuelProcessed.get(a);
    }

    public int freePlayFuel(Alliance a) {
        return freePlayFuel.get(a);
    }

    public int fouls(Alliance a) {
        return fouls.get(a);
    }

    public int chuteCount(Alliance a) {
        return chute.get(a);
    }

    public int corralCount(Alliance a) {
        return corral.get(a);
    }

    /** Human player moves CORRAL stock into the CHUTE. */
    public void refillChute(Alliance a) {
        int moved = Math.min(corral.get(a), CHUTE_CAPACITY - chute.get(a));
        corral.merge(a, -moved, Integer::sum);
        chute.merge(a, moved, Integer::sum);
    }

    /** Opens the CHUTE DOOR: one FUEL rolls out every 0.15 s until empty or closed. */
    public void releaseChute(Alliance a) {
        releasing.put(a, true);
        nextRelease.put(a, arena.time());
    }

    public void closeChute(Alliance a) {
        releasing.put(a, false);
    }

    /** Human player throws one stored FUEL over the alliance wall at the HUB, with human noise. */
    public void throwAtHub(Alliance a) {
        if (corral.get(a) > 0) {
            corral.merge(a, -1, Integer::sum);
        } else if (chute.get(a) > 0) {
            chute.merge(a, -1, Integer::sum);
        } else {
            return;
        }
        Vec3 release = new Vec3(-0.40, 1.20, 1.70);
        Vec3 target = RebuiltField.BLUE_HUB_CENTER.plus(new Vec3(0, 0, 0.30));
        double dx = target.x() - release.x();
        double dy = target.y() - release.y();
        double d = Math.hypot(dx, dy);
        double dh = target.z() - release.z();
        double pitch = Math.toRadians(55);
        double cos = Math.cos(pitch);
        double v = Math.sqrt(9.81 * d * d / (2 * cos * cos * (d * Math.tan(pitch) - dh)));
        // ASSUMPTION: 8% extra speed makes up for drag.
        v *= 1.08 * (1 + 0.04 * random.nextGaussian());
        double yaw = Math.atan2(dy, dx) + Math.toRadians(2 * random.nextGaussian());
        pitch += Math.toRadians(2 * random.nextGaussian());
        Vec3 velocity = new Vec3(v * Math.cos(pitch) * Math.cos(yaw), v * Math.cos(pitch) * Math.sin(yaw),
                v * Math.sin(pitch));
        spawn(a, release, velocity);
    }

    /** Called for every launched FUEL; G407 requires bumpers partly in the alliance's own zone. */
    public void onLaunch(Alliance a, Pose3 robotPose, double hx, double hy) {
        if (phase == MatchPhase.FREE_PLAY || phase == MatchPhase.POST_MATCH) {
            return;
        }
        double yaw = robotPose.rotation().yaw();
        double reach = Math.abs(Math.cos(yaw)) * hx + Math.abs(Math.sin(yaw)) * hy;
        double x = robotPose.position().x();
        boolean outside = a == Alliance.BLUE
                ? x - reach > RebuiltField.STARTING_LINE_X
                : x + reach < RebuiltField.LENGTH - RebuiltField.STARTING_LINE_X;
        if (outside) {
            fouls.merge(a, 1, Integer::sum);
            score.merge(a.opponent(), FOUL_MAJOR, Integer::sum);
            lastFoul = String.format("G407 %s launched from x=%.2f", a, x);
        }
    }

    /** Every FUEL to draw: live bodies plus CHUTE and CORRAL stock (AdvantageScope hides its staged FUEL). */
    public List<Pose3> fuelDisplayPoses() {
        List<Pose3> out = new ArrayList<>(arena.gamePieces().poses(RebuiltFuel.TYPE));
        for (Alliance a : Alliance.values()) {
            List<Vec3> chuteSlots = RebuiltFuelLayout.chuteDisplay(a);
            for (int i = 0; i < Math.min(chute.get(a), chuteSlots.size()); i++) {
                out.add(new Pose3(chuteSlots.get(i), Quat.IDENTITY));
            }
            List<Vec3> corralSlots = RebuiltFuelLayout.corralDisplay(a);
            for (int i = 0; i < Math.min(corral.get(a), corralSlots.size()); i++) {
                out.add(new Pose3(corralSlots.get(i), Quat.IDENTITY));
            }
        }
        return out;
    }

    public String lastFoul() {
        return lastFoul;
    }
}
