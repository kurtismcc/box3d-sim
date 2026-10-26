package box3dsim.season.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.arena.Alliance;
import box3dsim.arena.Arena;
import box3dsim.arena.ArenaConfig;
import box3dsim.gamepiece.GamePiece;
import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RebuiltMatchTest {
    private static final DsState AUTO = new DsState(true, true, false);
    private static final DsState TELEOP = new DsState(true, false, true);

    private Arena arena;
    private RebuiltMatch match;

    @BeforeEach
    void setUp() {
        arena = new Arena(ArenaConfig.defaults());
        RebuiltFieldHandles field = RebuiltField.build(arena.world());
        match = new RebuiltMatch(arena, field, new RebuiltMatch.Config(1, false, false, 8));
        arena.addParticipant(match);
        arena.reset();
    }

    @AfterEach
    void tearDown() {
        arena.close();
    }

    private void advanceTo(double t) {
        arena.advance(t - arena.time());
    }

    private GamePiece dropIntoHub(Alliance a) {
        Vec3 c = RebuiltField.hubCenter(a);
        return arena.gamePieces().spawn(RebuiltFuel.TYPE, new Vec3(c.x(), c.y(), 2.5), Vec3.ZERO, Vec3.ZERO, true);
    }

    /** Runs AUTO from t=0, then starts TELEOP at t=25 (disabled in between, as FMS would). */
    private void startTeleopAt25() {
        advanceTo(21);
        match.setDsState(DsState.DISABLED);
        advanceTo(25);
        match.setDsState(TELEOP);
    }

    @Test
    void timelineFollowsTheManual() {
        match.setDsState(AUTO);
        advanceTo(19.9);
        assertEquals(MatchPhase.AUTO, match.phase());
        advanceTo(20.1);
        assertEquals(MatchPhase.AUTO_TO_TELEOP, match.phase());
        assertFalse(match.outputsAllowed());
        startTeleopAt25();
        advanceTo(30);
        assertEquals(MatchPhase.TRANSITION, match.phase());
        assertTrue(match.outputsAllowed());
        advanceTo(36);
        assertEquals(MatchPhase.SHIFT_1, match.phase());
        advanceTo(125);
        assertEquals(MatchPhase.SHIFT_4, match.phase());
        advanceTo(136);
        assertEquals(MatchPhase.END_GAME, match.phase());
        advanceTo(166);
        assertEquals(MatchPhase.POST_MATCH, match.phase());
        assertFalse(match.outputsAllowed());
    }

    @Test
    void autoWinnerGoesInactiveFirst() {
        match.setDsState(AUTO);
        arena.advance(0.1);
        for (int i = 0; i < 3; i++) {
            dropIntoHub(Alliance.BLUE);
            arena.advance(0.5);
        }
        dropIntoHub(Alliance.RED);
        startTeleopAt25();
        assertEquals(3, match.autoFuel(Alliance.BLUE));
        assertEquals(1, match.autoFuel(Alliance.RED));
        assertEquals(Alliance.BLUE, match.firstInactive().orElseThrow());
        double[][] checks = {{30, 1, 1}, {40, 0, 1}, {65, 1, 0}, {90, 0, 1}, {115, 1, 0}, {140, 1, 1}};
        for (double[] c : checks) {
            advanceTo(c[0]);
            assertEquals(c[1] == 1, match.hubActive(Alliance.BLUE), "blue at " + c[0]);
            assertEquals(c[2] == 1, match.hubActive(Alliance.RED), "red at " + c[0]);
        }
    }

    @Test
    void scoringIsDelayedAndFuelReturns() {
        match.setDsState(TELEOP);
        arena.advance(0.1);
        int before = arena.gamePieces().count(RebuiltFuel.TYPE);
        double t0 = arena.time();
        dropIntoHub(Alliance.BLUE);
        advanceTo(t0 + 0.5);
        assertEquals(0, match.score(Alliance.BLUE));
        advanceTo(t0 + 2.5);
        assertEquals(1, match.score(Alliance.BLUE));
        assertEquals(before + 1, arena.gamePieces().count(RebuiltFuel.TYPE));
        boolean returned = arena.gamePieces().all().stream().anyMatch(p -> p.pose().position().x() > 5.2);
        assertTrue(returned);
    }

    @Test
    void inactiveHubHasThreeSecondGrace() {
        match.setDsState(AUTO);
        arena.advance(0.1);
        dropIntoHub(Alliance.BLUE);
        startTeleopAt25();
        arena.advance(0.1);
        assertEquals(Alliance.BLUE, match.firstInactive().orElseThrow());
        int base = match.score(Alliance.BLUE);
        advanceTo(25 + 9.0);
        dropIntoHub(Alliance.BLUE);
        advanceTo(25 + 13.5);
        assertEquals(base + 1, match.score(Alliance.BLUE), "scored within the grace period");
        int processed = match.fuelProcessed(Alliance.BLUE);
        advanceTo(25 + 14.0);
        dropIntoHub(Alliance.BLUE);
        advanceTo(25 + 17.0);
        assertEquals(base + 1, match.score(Alliance.BLUE), "no points after the grace period");
        assertEquals(processed + 1, match.fuelProcessed(Alliance.BLUE));
    }

    @Test
    void freePlayCountsWithoutPoints() {
        dropIntoHub(Alliance.RED);
        arena.advance(2.5);
        assertEquals(1, match.freePlayFuel(Alliance.RED));
        assertEquals(0, match.score(Alliance.RED));
        assertEquals(1, arena.gamePieces().count(RebuiltFuel.TYPE));
    }

    @Test
    void corralRefillAndChuteRelease() {
        arena.gamePieces().spawn(RebuiltFuel.TYPE, new Vec3(-0.5, 0.666, 0.3), Vec3.ZERO, Vec3.ZERO, true);
        arena.advance(1);
        assertEquals(1, match.corralCount(Alliance.BLUE));
        match.refillChute(Alliance.BLUE);
        assertEquals(25, match.chuteCount(Alliance.BLUE));
        assertEquals(0, match.corralCount(Alliance.BLUE));
        match.releaseChute(Alliance.BLUE);
        arena.advance(3.7);
        assertTrue(match.chuteCount(Alliance.BLUE) <= 1, "left " + match.chuteCount(Alliance.BLUE));
        arena.advance(0.2);
        assertEquals(0, match.chuteCount(Alliance.BLUE));
        assertEquals(25, arena.gamePieces().count(RebuiltFuel.TYPE));
    }

    @Test
    void humanPlayerThrowsReachTheHub() {
        match.setDsState(TELEOP);
        for (int i = 0; i < 20; i++) {
            match.throwAtHub(Alliance.BLUE);
            arena.advance(0.3);
        }
        arena.advance(4);
        System.out.printf("human player: %d of 20 throws processed by the hub%n", match.fuelProcessed(Alliance.BLUE));
        assertTrue(match.fuelProcessed(Alliance.BLUE) >= 6, "processed " + match.fuelProcessed(Alliance.BLUE));
    }

    @Test
    void launchingIntoOwnHubFromOutsideTheZoneIsAMajorFoul() {
        match.setDsState(TELEOP);
        arena.advance(0.1);
        // From midfield into the blue HUB: foul once the FUEL enters.
        GamePiece intoHub = dropIntoHub(Alliance.BLUE);
        match.onLaunch(Alliance.BLUE, intoHub, Pose3.ofXYYaw(6.0, 4, 0, 0), 0.45, 0.45);
        assertEquals(0, match.fouls(Alliance.BLUE), "no foul until the FUEL reaches the HUB");
        arena.advance(1);
        assertEquals(1, match.fouls(Alliance.BLUE));
        int redAfterFoul = match.score(Alliance.RED);
        assertTrue(redAfterFoul >= 15);
        // From midfield onto the floor (passing FUEL to your side): legal.
        GamePiece pass = arena.gamePieces().spawn(RebuiltFuel.TYPE, new Vec3(6.5, 2.0, 0.5), new Vec3(-3, 0, 1),
                Vec3.ZERO, true);
        match.onLaunch(Alliance.BLUE, pass, Pose3.ofXYYaw(7.0, 2.0, 0, Math.PI), 0.45, 0.45);
        // From inside the zone into the HUB: legal.
        GamePiece legal = dropIntoHub(Alliance.BLUE);
        match.onLaunch(Alliance.BLUE, legal, Pose3.ofXYYaw(3.0, 4, 0, 0), 0.45, 0.45);
        arena.advance(3);
        assertEquals(1, match.fouls(Alliance.BLUE));
        // From midfield into the opponent's HUB: not G407 (it's not "their" HUB).
        GamePiece other = dropIntoHub(Alliance.RED);
        match.onLaunch(Alliance.BLUE, other, Pose3.ofXYYaw(9.0, 4, 0, Math.PI), 0.45, 0.45);
        arena.advance(1);
        assertEquals(1, match.fouls(Alliance.BLUE));
    }

    @Test
    void freePlayNeverEndsOrCutsOutputs() {
        match.setDsState(TELEOP);
        arena.advance(1);
        assertEquals(MatchPhase.TRANSITION, match.phase());
        match.setFreePlay(true);
        arena.advance(150);
        assertEquals(MatchPhase.FREE_PLAY, match.phase());
        assertTrue(match.outputsAllowed());
        dropIntoHub(Alliance.BLUE);
        arena.advance(2.5);
        assertEquals(1, match.freePlayFuel(Alliance.BLUE));
        assertEquals(0, match.score(Alliance.BLUE));
        // Off again: the next enable starts a timed match.
        match.setFreePlay(false);
        match.setDsState(DsState.DISABLED);
        arena.advance(0.1);
        assertEquals(MatchPhase.FREE_PLAY, match.phase());
        match.setDsState(TELEOP);
        arena.advance(0.1);
        assertEquals(MatchPhase.TRANSITION, match.phase());
    }

    @Test
    void displayIncludesStoredFuel() {
        try (Arena full = new Arena(ArenaConfig.defaults())) {
            RebuiltMatch m = new RebuiltMatch(full, RebuiltField.build(full.world()), RebuiltMatch.Config.defaults());
            full.addParticipant(m);
            full.reset();
            assertEquals(360 + 48 + 48, m.fuelDisplayPoses().size());
        }
    }
}
