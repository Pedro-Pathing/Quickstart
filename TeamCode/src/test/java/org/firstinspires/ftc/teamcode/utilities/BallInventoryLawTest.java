package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.math.BigInteger;
import org.junit.jupiter.api.Test;

/** The pure functions of {@link BallInventory}, which OpenJML proves total and bounded. */
class BallInventoryLawTest {
  private static final long[] EDGES = {
    Long.MIN_VALUE,
    Long.MIN_VALUE + 1,
    -1L << 40,
    -41,
    -1,
    0,
    1,
    40,
    41,
    1L << 40,
    Long.MAX_VALUE - 1,
    Long.MAX_VALUE
  };

  @Test
  void elapsedMatchesUnboundedArithmeticOnEveryEdgeCombination() {
    for (long now : EDGES) {
      for (long since : EDGES) {
        for (long debounce : EDGES) {
          boolean oracle =
              BigInteger.valueOf(now)
                      .subtract(BigInteger.valueOf(since))
                      .compareTo(BigInteger.valueOf(debounce))
                  >= 0;
          assertEquals(
              oracle,
              BallInventory.elapsed(now, since, debounce),
              "now=" + now + " since=" + since + " debounce=" + debounce);
        }
      }
    }
  }

  @Test
  void aHugeGapIsNotMistakenForNoTimeHavingPassed() {
    // A plain nowMs - sinceMs wraps negative here.
    assertTrue(BallInventory.elapsed(Long.MAX_VALUE, Long.MIN_VALUE, 40));
  }

  @Test
  void mayCollectIsExactlyNotFull() {
    for (boolean known : new boolean[] {true, false}) {
      for (int c = -1; c <= 6; c++) {
        assertEquals(!(known && c >= 4), BallInventory.mayCollect(known, c, 4));
        assertEquals(!BallInventory.isFull(known, c, 4), BallInventory.mayCollect(known, c, 4));
      }
    }
    assertTrue(BallInventory.mayCollect(false, 99, 4), "unknown never blocks the intake");
    assertFalse(BallInventory.mayCollect(true, 4, 4));
    assertFalse(BallInventory.mayCollect(true, 5, 4), "over the cap still blocks");
  }

  @Test
  void ceilingIsCapacityPlusOneAndSaturates() {
    assertEquals(5, BallInventory.ceiling(4));
    assertEquals(Integer.MAX_VALUE, BallInventory.ceiling(Integer.MAX_VALUE));
    assertEquals(Integer.MAX_VALUE, BallInventory.clampCount(Integer.MAX_VALUE, Integer.MAX_VALUE));
  }

  @Test
  void clampCountStaysInRange() {
    assertEquals(0, BallInventory.clampCount(Integer.MIN_VALUE, 4));
    assertEquals(5, BallInventory.clampCount(Integer.MAX_VALUE, 4));
    assertEquals(3, BallInventory.clampCount(3, 4));
  }

  @Test
  void afterEntryNeverChangesAnUnknownCountAndNeverOverflows() {
    assertEquals(2, BallInventory.afterEntry(2, false, false, 4));
    assertEquals(2, BallInventory.afterEntry(2, false, true, 4));
    assertEquals(5, BallInventory.afterEntry(5, true, false, 4));
    assertEquals(0, BallInventory.afterEntry(0, true, true, 4));
    assertEquals(
        Integer.MAX_VALUE,
        BallInventory.afterEntry(Integer.MAX_VALUE, true, false, Integer.MAX_VALUE));
    assertEquals(0, BallInventory.afterEntry(Integer.MIN_VALUE + 1, true, true, 4));
  }

  @Test
  void afterShotStopsAtZero() {
    assertEquals(0, BallInventory.afterShot(0));
    assertEquals(2, BallInventory.afterShot(3));
  }

  @Test
  void anElementCountsOncePerBlockAndNeverOnTheBlockStartTimestampWrap() {
    assertFalse(BallInventory.countsNow(false, false, 100, 0, 40));
    assertFalse(BallInventory.countsNow(true, true, 100, 0, 40));
    assertTrue(BallInventory.countsNow(true, false, 40, 0, 40));
    assertFalse(BallInventory.countsNow(true, false, 39, 0, 40));
    assertEquals(7, BallInventory.blockStart(true, 7, 99));
    assertEquals(99, BallInventory.blockStart(false, 7, 99));
  }

  @Test
  void anExtremeTimestampStillCountsOnceThroughTheInstance() {
    var inv = new BallInventory(4, true);
    assertFalse(inv.updateEntrySensor(true, Long.MIN_VALUE, 40));
    assertTrue(inv.updateEntrySensor(true, Long.MAX_VALUE, 40));
    assertFalse(inv.updateEntrySensor(true, Long.MAX_VALUE, 40), "once per block");
    assertEquals(1, inv.count());
  }
}
