package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The HIVE belief state machine, driven by a fake clock. It decides which CELL is a legal target,
 * so its failure modes are shots into a downward-facing CELL (G417) or a freeze on a TIP it missed.
 */
public class HiveTrackerTest {

  private static final int CONFIRM = 3;
  private static final long SETTLE = 1500;
  private static final long MAX_AGE = 400;

  private long now;
  private HiveTracker hive;

  @BeforeEach
  public void setUp() {
    now = 10_000;
    hive = new HiveTracker(CONFIRM, SETTLE, MAX_AGE, () -> now);
  }

  /** Observes a CELL B reading as upward: evidence that a TIP has happened. */
  private boolean seeBUp() {
    return hive.observe(HiveCell.B, true, ++now);
  }

  /** Observes CELL A as upward: agrees with the match-start belief. */
  private boolean seeAUp() {
    return hive.observe(HiveCell.A, true, ++now);
  }

  @Test
  public void testStartsWithCellAUpAndNothingHappened() {
    assertEquals(HiveCell.A, hive.upCell());
    assertEquals(0, hive.tips());
    assertEquals(0, hive.epoch());
    assertEquals(HiveTracker.Source.MATCH_START, hive.source());
    assertTrue(hive.isSettled(), "nothing has moved, so nothing is swinging");
    assertEquals(-1, hive.msSinceVisionAgreed(), "camera has never confirmed anything");
    assertEquals(0, hive.pendingDisagreements());
  }

  @Test
  public void testRejectsAConfirmationCountBelowOne() {
    assertThrows(
        IllegalArgumentException.class, () -> new HiveTracker(0, SETTLE, MAX_AGE, () -> now));
    assertThrows(
        IllegalArgumentException.class, () -> new HiveTracker(-2, SETTLE, MAX_AGE, () -> now));
    // One is allowed: trust every fresh frame.
    new HiveTracker(1, SETTLE, MAX_AGE, () -> now);
  }

  // -------------------------------------------------------------------------------------------
  // Vision

  @Test
  public void testAgreeingObservationsChangeNothingButRecordTheConfirmation() {
    now = 20_000;
    assertFalse(hive.observe(HiveCell.A, true, now - 100));

    assertEquals(HiveCell.A, hive.upCell());
    assertEquals(0, hive.epoch());
    assertEquals(0, hive.tips());
    assertEquals(100, hive.msSinceVisionAgreed(), "confirmed 100 ms ago");

    now = 20_250;
    assertEquals(350, hive.msSinceVisionAgreed());
  }

  @Test
  public void testTheOtherCellSeenDownwardIsAlsoAgreement() {
    // Seeing B hanging down means A is up: the same belief, from the other CELL's tags.
    assertFalse(hive.observe(HiveCell.B, false, now));
    assertEquals(HiveCell.A, hive.upCell());
    assertEquals(0, hive.pendingDisagreements());
    assertEquals(0, hive.msSinceVisionAgreed());
  }

  @Test
  public void testTipIsOnlyAcceptedAfterConfirmFramesConsecutiveDisagreements() {
    assertFalse(seeBUp());
    assertEquals(1, hive.pendingDisagreements());
    assertEquals(HiveCell.A, hive.upCell(), "still believes A");
    now += 20;
    assertFalse(seeBUp());
    assertEquals(2, hive.pendingDisagreements());
    assertEquals(0, hive.epoch());

    now += 20;
    assertTrue(seeBUp(), "third consecutive frame flips the belief");

    assertEquals(HiveCell.B, hive.upCell());
    assertEquals(1, hive.tips());
    assertEquals(1, hive.epoch());
    assertEquals(HiveTracker.Source.VISION, hive.source());
    assertEquals(0, hive.msSinceVisionAgreed(), "the new belief is vision-confirmed right now");
    assertEquals(0, hive.pendingDisagreements());
  }

  @Test
  public void testAnAgreeingFrameBetweenDisagreementsResetsTheCount() {
    assertFalse(seeBUp());
    assertFalse(seeBUp());
    // One mis-ranged frame mid-swing must not be allowed to add up with the earlier ones.
    assertFalse(seeAUp());
    assertEquals(0, hive.pendingDisagreements());
    assertFalse(seeBUp());
    assertFalse(seeBUp());
    assertEquals(HiveCell.A, hive.upCell());
    assertEquals(0, hive.epoch());
    assertTrue(seeBUp());
    assertEquals(HiveCell.B, hive.upCell());
  }

  @Test
  public void testDisagreementCanComeFromEitherCellsTags() {
    // A seen downward and B seen upward both imply B is up; they add up.
    assertFalse(hive.observe(HiveCell.A, false, ++now));
    assertFalse(hive.observe(HiveCell.B, true, ++now));
    assertTrue(hive.observe(HiveCell.A, false, ++now));
    assertEquals(HiveCell.B, hive.upCell());
  }

  @Test
  public void testStaleObservationsAreIgnoredEntirely() {
    // Older than max age: ignored, so it neither counts toward a flip nor resets the count.
    assertFalse(seeBUp());
    assertFalse(seeBUp());
    assertEquals(2, hive.pendingDisagreements());

    assertFalse(hive.observe(HiveCell.B, true, now - MAX_AGE - 1));
    assertFalse(
        hive.observe(HiveCell.A, true, now - 5_000),
        "a stale agreeing frame must not reset the count");
    assertEquals(2, hive.pendingDisagreements());
    assertEquals(-1, hive.msSinceVisionAgreed());

    assertTrue(seeBUp(), "the next fresh frame still completes the run");
  }

  @Test
  public void testStaleBoundaryIsInclusive() {
    // Exactly max age old is still accepted; one millisecond older is not.
    HiveTracker one = new HiveTracker(1, SETTLE, MAX_AGE, () -> now);
    assertFalse(one.observe(HiveCell.B, true, now - MAX_AGE - 1));
    assertEquals(HiveCell.A, one.upCell());
    assertTrue(one.observe(HiveCell.B, true, now - MAX_AGE));
    assertEquals(HiveCell.B, one.upCell());
  }

  @Test
  public void testSingleFrameConfirmationFlipsImmediately() {
    HiveTracker eager = new HiveTracker(1, SETTLE, MAX_AGE, () -> now);
    assertTrue(eager.observe(HiveCell.B, true, now));
    assertEquals(HiveCell.B, eager.upCell());
    assertEquals(1, eager.tips());
  }

  @Test
  public void testTipsCountEveryFlipIncludingTheReturnToA() {
    for (int i = 1; i <= 4; i++) {
      HiveCell expectedUp = i % 2 == 1 ? HiveCell.B : HiveCell.A;
      for (int frame = 0; frame < CONFIRM; frame++) {
        hive.observe(expectedUp, true, ++now);
      }
      assertEquals(i, hive.tips(), "tip " + i);
      assertEquals(expectedUp, hive.upCell());
      assertEquals(i, hive.epoch());
      now += 5_000;
    }
  }

  @Test
  public void testObservationAgreeingWithTheNewBeliefResetsTheCandidate() {
    for (int i = 0; i < CONFIRM; i++) {
      seeBUp();
    }
    assertEquals(HiveCell.B, hive.upCell());
    // A lone frame of the old state is not enough to go back.
    assertFalse(seeAUp());
    assertFalse(seeAUp());
    assertEquals(2, hive.pendingDisagreements());
    assertFalse(seeBUp());
    assertEquals(0, hive.pendingDisagreements());
    assertEquals(HiveCell.B, hive.upCell());
  }

  // -------------------------------------------------------------------------------------------
  // Settling

  @Test
  public void testHiveIsUnsettledForSettleMsAfterAnyChange() {
    hive.recordTip();
    assertFalse(hive.isSettled());

    now += SETTLE - 1;
    assertFalse(hive.isSettled(), "still swinging");
    now += 1;
    assertTrue(hive.isSettled(), "exactly settleMs after the change");
  }

  @Test
  public void testEveryKindOfChangeRestartsTheSettleTimer() {
    hive.recordTip();
    now += SETTLE;
    assertTrue(hive.isSettled());

    hive.undoTip();
    assertFalse(hive.isSettled());
    now += SETTLE;
    assertTrue(hive.isSettled());

    for (int i = 0; i < CONFIRM; i++) {
      seeBUp();
    }
    assertFalse(hive.isSettled(), "a vision-detected TIP also needs to settle");
  }

  @Test
  public void testAgreeingFramesDoNotUnsettleTheHive() {
    seeAUp();
    seeAUp();
    assertTrue(hive.isSettled());
    seeBUp(); // a disagreement that has not been confirmed yet is not a change either
    assertTrue(hive.isSettled());
  }

  @Test
  public void testZeroSettleTimeIsAlwaysSettled() {
    HiveTracker instant = new HiveTracker(1, 0, MAX_AGE, () -> now);
    instant.recordTip();
    assertTrue(instant.isSettled());
  }

  // -------------------------------------------------------------------------------------------
  // Operator

  @Test
  public void testRecordTipFlipsTheCellAndBumpsEverything() {
    hive.recordTip();

    assertEquals(HiveCell.B, hive.upCell());
    assertEquals(1, hive.tips());
    assertEquals(1, hive.epoch());
    assertEquals(HiveTracker.Source.DRIVER, hive.source());

    hive.recordTip();
    assertEquals(HiveCell.A, hive.upCell());
    assertEquals(2, hive.tips());
    assertEquals(2, hive.epoch());
  }

  @Test
  public void testUndoTipRestoresThePreviousCellAndStillBumpsTheEpoch() {
    hive.recordTip();
    hive.recordTip();
    hive.recordTip();
    assertEquals(HiveCell.B, hive.upCell());
    assertEquals(3, hive.tips());
    int epochBefore = hive.epoch();

    hive.undoTip();

    assertEquals(HiveCell.A, hive.upCell());
    assertEquals(2, hive.tips());
    assertTrue(
        hive.epoch() > epochBefore,
        "a correction is a change: solutions solved for the wrong CELL must expire");
    assertEquals(HiveTracker.Source.DRIVER, hive.source());
  }

  @Test
  public void testUndoTipNeverDrivesTheTipCountNegative() {
    hive.undoTip();
    assertEquals(0, hive.tips());
    hive.undoTip();
    hive.undoTip();
    assertEquals(0, hive.tips());
    assertEquals(HiveCell.A, hive.upCell());
    assertEquals(0, hive.epoch());
    assertEquals(HiveTracker.Source.MATCH_START, hive.source());
    assertTrue(hive.isSettled());
  }

  @Test
  public void testUndoPastTheFirstTipCannotSelectTheDownwardCell() {
    hive.recordTip();
    hive.undoTip();
    int epoch = hive.epoch();
    now += SETTLE;
    hive.undoTip();
    assertEquals(HiveCell.A, hive.upCell());
    assertEquals(0, hive.tips());
    assertEquals(epoch, hive.epoch());
    assertTrue(hive.isSettled());
  }

  @Test
  public void testRestoreSetsBeliefFromAHandoffAndInvalidatesOldSolutions() {
    int epochBefore = hive.epoch();

    hive.restore(HiveCell.B, 3);

    assertEquals(HiveCell.B, hive.upCell());
    assertEquals(3, hive.tips());
    assertEquals(HiveTracker.Source.HANDOFF, hive.source());
    assertTrue(hive.epoch() > epochBefore);
  }

  @Test
  public void testRestoreClampsNegativeTipsAndClearsPendingDisagreements() {
    seeBUp();
    seeBUp();
    assertEquals(2, hive.pendingDisagreements());

    hive.restore(HiveCell.A, -4);

    assertEquals(0, hive.tips());
    assertEquals(0, hive.pendingDisagreements());
    assertEquals(HiveCell.A, hive.upCell());
  }

  @Test
  public void testRestoreDoesNotStartTheSettleTimer() {
    hive.restore(HiveCell.B, 1);
    assertTrue(hive.isSettled(), "a restored belief is a resting HIVE, not a swinging one");
  }

  @Test
  public void testMsSinceVisionAgreedKeepsTheNewestObservationOnly() {
    now = 50_000;
    hive.observe(HiveCell.A, true, now - 300);
    hive.observe(HiveCell.A, true, now - 100);
    // An older frame arriving late must not move the record backwards.
    hive.observe(HiveCell.A, true, now - 350);
    assertEquals(100, hive.msSinceVisionAgreed());
  }

  @Test
  public void testFromConfigUsesTheConfiguredThresholds() {
    HiveTracker configured = HiveTracker.fromConfig(() -> now);
    int frames = config.hive.confirm_frames_count;

    for (int i = 0; i < frames - 1; i++) {
      assertFalse(configured.observe(HiveCell.B, true, ++now));
    }
    assertTrue(configured.observe(HiveCell.B, true, ++now));
    assertFalse(configured.isSettled());
    now += config.hive.settle_ms;
    assertTrue(configured.isSettled());

    // And the configured freshness window applies.
    HiveTracker fresh = HiveTracker.fromConfig(() -> now);
    assertFalse(fresh.observe(HiveCell.B, true, now - config.hive.max_observation_age_ms - 1));
    assertEquals(0, fresh.pendingDisagreements());
    fresh.observe(HiveCell.B, true, now - config.hive.max_observation_age_ms);
    assertEquals(1, fresh.pendingDisagreements());
  }

  @Test
  public void testConfigDefaultsAreSensible() {
    assertTrue(config.hive.confirm_frames_count >= 2, "a single frame must not be enough");
    assertTrue(config.hive.settle_ms > 0);
    assertTrue(config.hive.max_observation_age_ms > 0);
    // The upward/downward split sits between the two CELL heights, with margin either side.
    double up = config.field.hive.up_opening_height_in;
    double down = config.field.hive.down_opening_height_in;
    assertTrue(config.hive.up_height_threshold_in < up - 5.0);
    assertTrue(config.hive.up_height_threshold_in > down + 5.0);
  }

  @Test
  public void testMultipleSightingsFromOneFrameCannotConfirmATip() {
    assertFalse(hive.observe(HiveCell.B, true, now));
    assertFalse(hive.observe(HiveCell.A, false, now));
    assertFalse(hive.observe(HiveCell.B, true, now));
    assertEquals(1, hive.pendingDisagreements());
    assertEquals(HiveCell.A, hive.upCell());
    assertFalse(hive.observe(HiveCell.B, true, now - 1));
    assertFalse(hive.observe(HiveCell.B, true, now + 1));
    now += 20;
    assertFalse(hive.observe(HiveCell.B, true, now));
    now += 20;
    assertTrue(hive.observe(HiveCell.A, false, now));
  }

  /**
   * The HIVE swings far enough for the camera to see CELL A low for a few frames, then rocks back
   * without tipping. That is zero TIPS, not two: the TIP count sets the human player's NECTAR
   * allowance, so a phantom pair would invite a G426 foul.
   */
  @Test
  public void aSwingThatRocksBackIsNotTwoTips() {
    long[] now = {10_000};
    HiveTracker tracker = new HiveTracker(3, 1500, 400, () -> now[0]);
    for (int i = 0; i < 3; i++) {
      now[0] += 33;
      tracker.observe(HiveCell.A, false, now[0]);
    }
    assertEquals(HiveCell.B, tracker.upCell());
    assertEquals(1, tracker.tips());
    for (int i = 0; i < 3; i++) {
      now[0] += 33;
      tracker.observe(HiveCell.A, true, now[0]);
    }
    assertEquals(HiveCell.A, tracker.upCell());
    assertEquals(0, tracker.tips(), "a stopped TIP must not count");
    assertEquals(2, tracker.epoch(), "both changes still invalidate shot solutions");
  }

  /** Once the HIVE has settled, a change back is a real second TIP. */
  @Test
  public void aChangeBackAfterSettlingIsARealTip() {
    long[] now = {10_000};
    HiveTracker tracker = new HiveTracker(3, 1500, 400, () -> now[0]);
    for (int i = 0; i < 3; i++) {
      now[0] += 33;
      tracker.observe(HiveCell.A, false, now[0]);
    }
    now[0] += 2_000;
    for (int i = 0; i < 3; i++) {
      now[0] += 33;
      tracker.observe(HiveCell.A, true, now[0]);
    }
    assertEquals(HiveCell.A, tracker.upCell());
    assertEquals(2, tracker.tips());
  }
}
