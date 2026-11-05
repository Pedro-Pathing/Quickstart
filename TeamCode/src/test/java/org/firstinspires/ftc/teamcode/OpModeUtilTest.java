package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.junit.jupiter.api.Test;

public class OpModeUtilTest {

  private static final long T0 = 1_000_000L;

  @Test
  public void teleOpDiscardsStaleAllianceAndPoseTogether() {
    for (Alliance alliance : Alliance.values()) {
      MatchProfile profile = config.loadMatchProfile(alliance);
      Map<String, Object> board = new HashMap<>();
      board.put("ALLIANCE", alliance);
      board.put(OpModeUtil.getBlackboardPoseKey(alliance), new Pose(50, 60, 1));
      OpModeUtil.saveHandoff(board, alliance, trackerAfter(1), 3, T0);
      for (long now : new long[] {T0 - 1, T0 + OpModeUtil.HANDOFF_MAX_AGE_MS + 1}) {
        var start = OpModeUtil.resolveTeleOpStart(board, profile, now);
        assertFalse(start.allianceConfirmed());
        assertFalse(start.poseRestored());
        assertSame(profile.startPose(), start.pose());
        assertNull(start.handoff());
      }
      board.remove(OpModeUtil.getBlackboardHandoffKey(alliance));
      var start = OpModeUtil.resolveTeleOpStart(board, profile, T0);
      assertFalse(start.allianceConfirmed());
      assertSame(profile.startPose(), start.pose());
    }
  }

  @Test
  public void teleOpRestoresAllianceAndPoseOnlyWithMatchingFreshHandoff() {
    for (Alliance alliance : Alliance.values()) {
      MatchProfile profile = config.loadMatchProfile(alliance);
      Map<String, Object> board = new HashMap<>();
      board.put("ALLIANCE", alliance);
      Pose saved = new Pose(50, 60, 1);
      board.put(OpModeUtil.getBlackboardPoseKey(alliance), saved);
      OpModeUtil.saveHandoff(board, alliance, trackerAfter(1), 3, T0);
      var start = OpModeUtil.resolveTeleOpStart(board, profile, T0 + OpModeUtil.HANDOFF_MAX_AGE_MS);
      assertTrue(start.allianceConfirmed());
      assertTrue(start.poseRestored());
      assertSame(saved, start.pose());
      assertEquals(3, start.handoff().inventory());
      board.put(OpModeUtil.getBlackboardPoseKey(alliance), "invalid pose");
      assertFalse(OpModeUtil.resolveTeleOpStart(board, profile, T0).poseRestored());
      assertSame(profile.startPose(), OpModeUtil.resolveTeleOpStart(board, profile, T0).pose());
      board.put("ALLIANCE", alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED);
      assertFalse(OpModeUtil.resolveTeleOpStart(board, profile, T0).allianceConfirmed());
      board.remove("ALLIANCE");
      assertFalse(OpModeUtil.resolveTeleOpStart(board, profile, T0).allianceConfirmed());
    }
  }

  /** A tracker that has seen {@code tips} TIPS, so its upward CELL and counter are known. */
  private static HiveTracker trackerAfter(int tips) {
    HiveTracker hive = new HiveTracker(3, 1500, 400, () -> T0);
    for (int i = 0; i < tips; i++) {
      hive.recordTip();
    }
    return hive;
  }

  @Test
  public void testBlackboardPoseKeys() {
    assertEquals("RED_POSE", OpModeUtil.getBlackboardPoseKey(Alliance.RED));
    assertEquals("BLUE_POSE", OpModeUtil.getBlackboardPoseKey(Alliance.BLUE));
  }

  @Test
  public void testBlackboardHandoffKeysAreDistinctPerAlliance() {
    assertEquals("RED_HANDOFF", OpModeUtil.getBlackboardHandoffKey(Alliance.RED));
    assertEquals("BLUE_HANDOFF", OpModeUtil.getBlackboardHandoffKey(Alliance.BLUE));
  }

  @Test
  public void testSavedPoseReturnsTheStoredPoseOrTheDefault() {
    Map<String, Object> board = new HashMap<>();
    Pose fallback = new Pose(1, 2, 3);

    assertSame(fallback, OpModeUtil.getSavedPose(board, Alliance.RED, fallback), "nothing saved");

    Pose red = new Pose(58, 27, Math.PI / 2);
    board.put(OpModeUtil.getBlackboardPoseKey(Alliance.RED), red);
    assertSame(red, OpModeUtil.getSavedPose(board, Alliance.RED, fallback));
    assertSame(
        fallback,
        OpModeUtil.getSavedPose(board, Alliance.BLUE, fallback),
        "the other alliance's pose is not ours");

    // Something else under the key (a stale value of the wrong type) is ignored, not cast.
    board.put(OpModeUtil.getBlackboardPoseKey(Alliance.BLUE), "not a pose");
    assertSame(fallback, OpModeUtil.getSavedPose(board, Alliance.BLUE, fallback));
  }

  @Test
  public void testHandoffRoundTripCarriesHiveStateAndInventory() {
    Map<String, Object> board = new HashMap<>();
    HiveTracker hive = trackerAfter(3);
    assertEquals(HiveCell.B, hive.upCell());

    OpModeUtil.saveHandoff(board, Alliance.BLUE, hive, 2, T0);

    OpModeUtil.Handoff handoff = OpModeUtil.getFreshHandoff(board, Alliance.BLUE, T0 + 5_000);
    assertNotNull(handoff);
    assertEquals(HiveCell.B, handoff.upCell());
    assertEquals(3, handoff.tips());
    assertEquals(2, handoff.inventory());
    assertEquals(T0, handoff.savedAtMs());
  }

  @Test
  public void testHandoffKeepsAnUnknownInventoryOfMinusOne() {
    Map<String, Object> board = new HashMap<>();

    OpModeUtil.saveHandoff(board, Alliance.RED, trackerAfter(0), -1, T0);

    OpModeUtil.Handoff handoff = OpModeUtil.getFreshHandoff(board, Alliance.RED, T0 + 1);
    assertNotNull(handoff);
    assertEquals(-1, handoff.inventory(), "-1 means the sender did not know");
    assertEquals(HiveCell.A, handoff.upCell());
    assertEquals(0, handoff.tips());
  }

  @Test
  public void testHandoffIsSeparatePerAlliance() {
    Map<String, Object> board = new HashMap<>();
    OpModeUtil.saveHandoff(board, Alliance.RED, trackerAfter(1), 4, T0);

    assertNotNull(OpModeUtil.getFreshHandoff(board, Alliance.RED, T0));
    assertNull(
        OpModeUtil.getFreshHandoff(board, Alliance.BLUE, T0),
        "a RED AUTO must never seed a BLUE TeleOp");
    assertNull(OpModeUtil.getFreshHandoff(new HashMap<>(), Alliance.RED, T0));
  }

  @Test
  public void testHandoffFreshnessWindowIsInclusiveAtTheLimit() {
    Map<String, Object> board = new HashMap<>();
    OpModeUtil.saveHandoff(board, Alliance.RED, trackerAfter(1), 4, T0);

    assertEquals(120_000L, OpModeUtil.HANDOFF_MAX_AGE_MS);
    assertNotNull(OpModeUtil.getFreshHandoff(board, Alliance.RED, T0));
    assertNotNull(
        OpModeUtil.getFreshHandoff(board, Alliance.RED, T0 + OpModeUtil.HANDOFF_MAX_AGE_MS),
        "exactly at the limit is still the same match");
    assertNull(
        OpModeUtil.getFreshHandoff(board, Alliance.RED, T0 + OpModeUtil.HANDOFF_MAX_AGE_MS + 1),
        "one millisecond past it is a different match");
  }

  @Test
  public void testHandoffFromTheFutureIsRejected() {
    // A clock that went backwards (a Control Hub reboot, say) must not resurrect an old handoff.
    Map<String, Object> board = new HashMap<>();
    OpModeUtil.saveHandoff(board, Alliance.RED, trackerAfter(1), 4, T0);

    assertNull(OpModeUtil.getFreshHandoff(board, Alliance.RED, T0 - 1));
    assertNull(OpModeUtil.getFreshHandoff(board, Alliance.RED, 0));
  }

  @Test
  public void testHandoffIgnoresAForeignObjectUnderItsKey() {
    Map<String, Object> board = new HashMap<>();
    board.put(OpModeUtil.getBlackboardHandoffKey(Alliance.RED), "garbage");
    assertNull(OpModeUtil.getFreshHandoff(board, Alliance.RED, T0));
  }

  @Test
  public void testHandoffRecordFreshnessMatchesTheLookup() {
    OpModeUtil.Handoff handoff = new OpModeUtil.Handoff(HiveCell.A, 0, 0, T0);

    assertTrue(handoff.isFresh(T0));
    assertTrue(handoff.isFresh(T0 + OpModeUtil.HANDOFF_MAX_AGE_MS));
    assertFalse(handoff.isFresh(T0 + OpModeUtil.HANDOFF_MAX_AGE_MS + 1));
    assertFalse(handoff.isFresh(T0 - 1));
  }

  @Test
  public void testSavingTwiceKeepsOnlyTheLatest() {
    Map<String, Object> board = new HashMap<>();
    HiveTracker hive = trackerAfter(0);
    OpModeUtil.saveHandoff(board, Alliance.RED, hive, 4, T0);
    hive.recordTip();
    OpModeUtil.saveHandoff(board, Alliance.RED, hive, 1, T0 + 10_000);

    OpModeUtil.Handoff handoff = OpModeUtil.getFreshHandoff(board, Alliance.RED, T0 + 10_000);
    assertNotNull(handoff);
    assertEquals(HiveCell.B, handoff.upCell());
    assertEquals(1, handoff.tips());
    assertEquals(1, handoff.inventory());
    assertEquals(T0 + 10_000, handoff.savedAtMs());
  }
}
