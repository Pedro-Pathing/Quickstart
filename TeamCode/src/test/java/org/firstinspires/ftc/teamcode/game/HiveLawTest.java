package org.firstinspires.ftc.teamcode.game;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.math.BigInteger;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import org.junit.jupiter.api.Test;

/**
 * Edge cases and exact-arithmetic oracles for {@link HiveLaw}. OpenJML proves the same contracts
 * for every input; these keep the proven code honest against an independent {@link BigInteger}
 * computation and pin the sentinel and overflow cases the tracker relies on.
 */
class HiveLawTest {

  private static BigInteger big(long v) {
    return BigInteger.valueOf(v);
  }

  @Property
  boolean subSaturatesLikeExactArithmetic(@ForAll long a, @ForAll long b) {
    BigInteger exact = big(a).subtract(big(b));
    BigInteger clamped = exact.max(big(Long.MIN_VALUE)).min(big(Long.MAX_VALUE));
    return big(HiveLaw.sub(a, b)).equals(clamped);
  }

  @Property
  boolean settledMatchesExactArithmetic(
      @ForAll long now, @ForAll long lastChange, @ForAll long settleMs) {
    BigInteger elapsed = big(now).subtract(big(lastChange));
    boolean expected = elapsed.compareTo(big(Math.max(0L, settleMs))) >= 0;
    return HiveLaw.settled(now, lastChange, settleMs) == expected;
  }

  @Property
  boolean acceptsMatchesExactArithmetic(
      @ForAll long observed, @ForAll long last, @ForAll long now, @ForAll long maxAge) {
    boolean expected =
        observed > last
            && observed <= now
            && big(now).subtract(big(observed)).compareTo(big(Math.max(0L, maxAge))) <= 0;
    return HiveLaw.accepts(observed, last, now, maxAge) == expected;
  }

  @Property
  boolean epochAlwaysDiffers(@ForAll int epoch) {
    return HiveLaw.nextEpoch(epoch) != epoch;
  }

  @Test
  void epochWrapsAtMaxValueAndStillDiffers() {
    assertEquals(Integer.MIN_VALUE, HiveLaw.nextEpoch(Integer.MAX_VALUE));
    assertEquals(1, HiveLaw.nextEpoch(0));
    assertEquals(0, HiveLaw.nextEpoch(-1));
  }

  @Test
  void neverChangedIsSettledEvenForAHugeClock() {
    // A raw now - NEVER overflows to a negative number once now exceeds 2^62.
    assertTrue(HiveLaw.settled(0, HiveLaw.NEVER, 500));
    assertTrue(HiveLaw.settled(Long.MAX_VALUE, HiveLaw.NEVER, 500));
    assertTrue(HiveLaw.settled(Long.MAX_VALUE, Long.MIN_VALUE, Long.MAX_VALUE));
  }

  @Test
  void settleWindowBoundary() {
    assertFalse(HiveLaw.settled(1_499, 1_000, 500));
    assertTrue(HiveLaw.settled(1_500, 1_000, 500));
    assertFalse(HiveLaw.settled(900, 1_000, 0), "clock behind the change is not settled");
    assertTrue(HiveLaw.settled(1_000, 1_000, -5), "a negative window counts as zero");
  }

  @Test
  void acceptsRejectsStaleDuplicateFutureAndOverflowingStamps() {
    assertTrue(HiveLaw.accepts(100, Long.MIN_VALUE, 100, 0));
    assertFalse(HiveLaw.accepts(100, 100, 100, 50), "repeat of the last accepted frame");
    assertFalse(HiveLaw.accepts(101, 0, 100, 50), "from the future");
    assertFalse(HiveLaw.accepts(49, 0, 100, 50), "older than the limit");
    assertTrue(HiveLaw.accepts(50, 0, 100, 50));
    // now - observed overflows here and wraps to a small negative number if done naively.
    assertFalse(HiveLaw.accepts(Long.MIN_VALUE + 1, Long.MIN_VALUE, 0, 1_000));
    assertTrue(HiveLaw.accepts(Long.MIN_VALUE + 1, Long.MIN_VALUE, 0, Long.MAX_VALUE));
  }

  @Test
  void candidateCountResetsRestartsAndIncrements() {
    // Agreeing frame resets whatever was pending.
    assertEquals(0, HiveLaw.nextCandidateCount(true, true, false, 3));
    // First disagreeing frame starts at 1.
    assertEquals(1, HiveLaw.nextCandidateCount(false, true, true, 0));
    // Continuing the same candidate adds one.
    assertEquals(2, HiveLaw.nextCandidateCount(false, true, false, 1));
    assertEquals(3, HiveLaw.nextCandidateCount(false, true, false, 2));
    // A different candidate restarts the run.
    assertEquals(1, HiveLaw.nextCandidateCount(false, true, true, 4));
    // Never wraps negative.
    assertEquals(
        Integer.MAX_VALUE, HiveLaw.nextCandidateCount(false, true, false, Integer.MAX_VALUE));
  }

  @Test
  void confirmationNeedsConfirmFramesConsecutiveDisagreements() {
    int count = 0;
    int confirmFrames = 3;
    for (int frame = 1; frame <= confirmFrames; frame++) {
      count = HiveLaw.nextCandidateCount(false, true, false, count);
      assertEquals(frame, count);
      assertEquals(frame >= confirmFrames, HiveLaw.confirmed(count, confirmFrames));
    }
    // One agreeing frame in the middle starts the run over.
    count = HiveLaw.nextCandidateCount(false, true, false, 2);
    count = HiveLaw.nextCandidateCount(true, true, false, count);
    assertFalse(HiveLaw.confirmed(HiveLaw.nextCandidateCount(false, true, false, count), 2));
  }

  @Test
  void tipArithmeticNeverGoesNegativeOrWraps() {
    assertEquals(0, HiveLaw.tipsAfterUndo(0));
    assertEquals(0, HiveLaw.tipsAfterUndo(-4));
    assertEquals(2, HiveLaw.tipsAfterUndo(3));
    assertEquals(Integer.MAX_VALUE, HiveLaw.tipsAfterRecord(Integer.MAX_VALUE));
    assertEquals(0, HiveLaw.restoredTips(-1));
    assertEquals(7, HiveLaw.restoredTips(7));
    assertEquals(0, HiveLaw.tipsAfterVision(true, 1));
    assertEquals(4, HiveLaw.tipsAfterVision(false, 3));
    assertEquals(1, HiveLaw.tipsAfterVision(true, 0), "no TIP to undo, so it counts as a new one");
    assertFalse(HiveLaw.canUndo(0));
    assertTrue(HiveLaw.canUndo(1));
  }

  @Test
  void everyTipFlipsTheUpCell() {
    assertNotEquals(true, HiveLaw.flipped(true));
    assertEquals(true, HiveLaw.flipped(false));
    // A sighting of A upward implies A up; of A downward implies B up; same for B.
    assertTrue(HiveLaw.impliedUpIsA(true, true));
    assertFalse(HiveLaw.impliedUpIsA(true, false));
    assertFalse(HiveLaw.impliedUpIsA(false, true));
    assertTrue(HiveLaw.impliedUpIsA(false, false));
  }

  @Test
  void rockedBackNeedsAVisionChangeInsideTheWindowAndATipToUndo() {
    assertTrue(HiveLaw.rockedBack(true, 1_200, 1_000, 500, 1));
    assertFalse(HiveLaw.rockedBack(false, 1_200, 1_000, 500, 1), "driver change");
    assertFalse(HiveLaw.rockedBack(true, 1_500, 1_000, 500, 1), "window elapsed");
    assertFalse(HiveLaw.rockedBack(true, 1_200, 1_000, 500, 0), "nothing to undo");
  }

  @Test
  void msSinceAgreedUsesTheSentinelNotAnOverflowingSubtraction() {
    assertEquals(-1, HiveLaw.msSinceAgreed(5_000, HiveLaw.NEVER));
    assertEquals(250, HiveLaw.msSinceAgreed(5_000, 4_750));
    assertEquals(Long.MAX_VALUE, HiveLaw.msSinceAgreed(Long.MAX_VALUE, Long.MIN_VALUE));
  }
}
