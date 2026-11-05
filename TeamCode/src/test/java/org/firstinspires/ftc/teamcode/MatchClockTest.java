package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.game.MatchClock;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** The time-dependent rules (G410/G426 FLOWER unlock, parking reminder) against a fake clock. */
public class MatchClockTest {

  private static final long AUTO_MS = 30_000;
  private static final long TELEOP_MS = 120_000;
  private static final long UNLOCK_REMAINING = 60_000;
  private static final long FINAL_REMAINING = 20_000;

  private long now;

  @BeforeEach
  public void setUp() {
    now = 500_000;
  }

  private MatchClock teleop() {
    return new MatchClock(
        MatchClock.Period.TELEOP, TELEOP_MS, UNLOCK_REMAINING, FINAL_REMAINING, () -> now);
  }

  private MatchClock auto() {
    return new MatchClock(
        MatchClock.Period.AUTO, AUTO_MS, UNLOCK_REMAINING, FINAL_REMAINING, () -> now);
  }

  @Test
  public void testBeforeStartTheFullPeriodRemainsAndNothingFires() {
    MatchClock clock = teleop();

    assertFalse(clock.isStarted());
    assertEquals(0, clock.elapsedMs());
    assertEquals(TELEOP_MS, clock.remainingMs());
    assertFalse(clock.isFlowerUnlocked());
    assertFalse(clock.isFinalWarning());
    assertNull(clock.pollCue());

    // Time passing before the drivers press play changes nothing.
    now += 1_000_000;
    assertEquals(0, clock.elapsedMs());
    assertEquals(TELEOP_MS, clock.remainingMs());
    assertNull(clock.pollCue());
  }

  @Test
  public void testStartAnchorsElapsedAndRemaining() {
    MatchClock clock = teleop();
    clock.start();

    assertTrue(clock.isStarted());
    assertEquals(MatchClock.Period.TELEOP, clock.period());
    assertEquals(0, clock.elapsedMs());
    assertEquals(TELEOP_MS, clock.remainingMs());

    now += 45_000;
    assertEquals(45_000, clock.elapsedMs());
    assertEquals(75_000, clock.remainingMs());
  }

  @Test
  public void testRemainingNeverGoesNegative() {
    MatchClock clock = teleop();
    clock.start();
    now += TELEOP_MS;
    assertEquals(0, clock.remainingMs());
    now += 500_000;
    assertEquals(0, clock.remainingMs());
    assertEquals(TELEOP_MS + 500_000, clock.elapsedMs());
  }

  @Test
  public void testClockGoingBackwardsDoesNotProduceNegativeElapsed() {
    MatchClock clock = teleop();
    clock.start();
    now -= 5_000;
    assertEquals(0, clock.elapsedMs());
    assertEquals(TELEOP_MS, clock.remainingMs());
  }

  @Test
  public void testFlowerUnlocksExactlyWhenTheFinalMinuteBegins() {
    MatchClock clock = teleop();
    clock.start();

    now += TELEOP_MS - UNLOCK_REMAINING - 1;
    assertEquals(UNLOCK_REMAINING + 1, clock.remainingMs());
    assertFalse(clock.isFlowerUnlocked(), "one millisecond early");

    now += 1;
    assertEquals(UNLOCK_REMAINING, clock.remainingMs());
    assertTrue(clock.isFlowerUnlocked(), "remaining <= 60 s");

    now += 10_000;
    assertTrue(clock.isFlowerUnlocked(), "stays unlocked to the end");
    now += 100_000;
    assertTrue(clock.isFlowerUnlocked());
  }

  @Test
  public void testFlowerIsNeverUnlockedInAuto() {
    MatchClock clock = auto();
    clock.start();
    for (long t : new long[] {0, 10_000, 29_999, 30_000, 99_000}) {
      now = 500_000 + t;
      assertFalse(clock.isFlowerUnlocked(), "AUTO at +" + t);
    }
  }

  @Test
  public void testFinalWarningIsReportedInBothPeriods() {
    MatchClock teleop = teleop();
    teleop.start();
    now += TELEOP_MS - FINAL_REMAINING - 1;
    assertFalse(teleop.isFinalWarning());
    now += 1;
    assertTrue(teleop.isFinalWarning());

    now = 700_000;
    MatchClock auto = auto();
    auto.start();
    now += AUTO_MS - FINAL_REMAINING;
    assertTrue(auto.isFinalWarning(), "AUTO is 30 s, so the last 20 s is its warning window");
  }

  @Test
  public void testTeleopCuesFireOnceEachInOrder() {
    MatchClock clock = teleop();
    clock.start();

    assertNull(clock.pollCue());
    now += 59_999; // 60.001 s left
    assertNull(clock.pollCue());

    now += 1; // exactly 60 s left
    assertEquals(MatchClock.Cue.FLOWER_UNLOCK, clock.pollCue());
    assertNull(clock.pollCue(), "fires once");
    assertNull(clock.pollCue());

    now += 40_000; // 20 s left: the park watch starts silently; the park alert decides later
    assertNull(clock.pollCue());

    now += 30_000;
    assertNull(clock.pollCue(), "nothing more to say after the buzzer");
  }

  @Test
  public void testLatePollStillDeliversTheUnlockOnce() {
    MatchClock clock = teleop();
    clock.start();
    now += TELEOP_MS - 5_000; // the loop stalled; the boundary is already behind us

    assertEquals(MatchClock.Cue.FLOWER_UNLOCK, clock.pollCue());
    assertNull(clock.pollCue());
  }

  @Test
  public void testAutoNeverRaisesACue() {
    MatchClock clock = auto();
    clock.start();
    for (int i = 0; i < 40; i++) {
      assertNull(clock.pollCue(), "AUTO +" + i + " s");
      now += 1_000;
    }
  }

  @Test
  public void testStartResetsTheCuesAndTheAnchor() {
    MatchClock clock = teleop();
    clock.start();
    now += TELEOP_MS;
    assertEquals(MatchClock.Cue.FLOWER_UNLOCK, clock.pollCue());

    // A second match on the same object (OpMode restarted) begins from scratch.
    clock.start();
    assertEquals(0, clock.elapsedMs());
    assertEquals(TELEOP_MS, clock.remainingMs());
    assertFalse(clock.isFlowerUnlocked());
    assertNull(clock.pollCue());
    now += TELEOP_MS - UNLOCK_REMAINING;
    assertEquals(MatchClock.Cue.FLOWER_UNLOCK, clock.pollCue());
  }

  @Test
  public void testParkAlertWaitsForTheWindowThenForDrivePlusMargin() {
    MatchClock clock = teleop();
    clock.start();
    double drive = 3.0; // seconds
    long margin = 4_000;

    now += TELEOP_MS - FINAL_REMAINING - 1; // 20.001 s left: window not open yet
    assertFalse(clock.isParkAlert(drive, margin));
    assertFalse(clock.isParkAlert(Double.POSITIVE_INFINITY, margin), "nothing before the window");

    now += 1; // 20 s left: window open, but 20 s > 3 s + 4 s
    assertFalse(clock.isParkAlert(drive, margin));

    now = 500_000 + TELEOP_MS - 7_001; // 7.001 s left
    assertFalse(clock.isParkAlert(drive, margin));
    now += 1; // exactly drive + margin left
    assertTrue(clock.isParkAlert(drive, margin));
    now += 7_000; // buzzer
    assertTrue(clock.isParkAlert(drive, margin));
  }

  @Test
  public void testParkAlertWithNoEstimateFiresAsSoonAsTheWindowOpens() {
    MatchClock clock = teleop();
    clock.start();
    now += TELEOP_MS - FINAL_REMAINING;
    assertTrue(clock.isParkAlert(Double.POSITIVE_INFINITY, 4_000), "no clear route");
    assertTrue(clock.isParkAlert(Double.NaN, 4_000), "no estimate");
    assertTrue(clock.isParkAlert(-1.0, 4_000), "nonsense estimate");
  }

  @Test
  public void testParkAlertNeverFiresBeforeStart() {
    MatchClock clock = teleop();
    now += TELEOP_MS * 2;
    assertFalse(clock.isParkAlert(Double.POSITIVE_INFINITY, 4_000));
  }

  @Test
  public void testAutoParkIsDueAtDrivePlusMarginOrTheReserveWhicheverIsEarlier() {
    MatchClock clock = auto();
    clock.start();
    long margin = 2_500;
    long reserve = 3_000;

    // Far away: 5.3 s drive + 2.5 s margin = 7.8 s before the end.
    now += AUTO_MS - 7_801;
    assertFalse(clock.isParkDue(5.3, margin, reserve));
    now += 1;
    assertTrue(clock.isParkDue(5.3, margin, reserve));

    // Close by: 0.3 s + 2.5 s = 2.8 s is later than the 3 s reserve, so the reserve wins.
    now = 500_000 + AUTO_MS - 3_001;
    assertFalse(clock.isParkDue(0.3, margin, reserve));
    now += 1;
    assertTrue(clock.isParkDue(0.3, margin, reserve));
  }

  @Test
  public void testAutoParkWithNoEstimateWaitsForTheReserveInsteadOfCancellingEarly() {
    MatchClock clock = auto();
    clock.start();
    now += AUTO_MS - 3_001;
    assertFalse(clock.isParkDue(Double.POSITIVE_INFINITY, 2_500, 3_000), "no clear route");
    assertFalse(clock.isParkDue(Double.NaN, 2_500, 3_000), "no estimate");
    assertFalse(clock.isParkDue(-1.0, 2_500, 3_000), "nonsense estimate");
    now += 1;
    assertTrue(clock.isParkDue(Double.NaN, 2_500, 3_000));
  }

  @Test
  public void testAutoParkIsNeverDueBeforeStart() {
    MatchClock clock = auto();
    now += AUTO_MS * 2;
    assertFalse(clock.isParkDue(100.0, 2_500, 3_000));
  }

  @Test
  public void testNectarAllowanceIsOnePerTipUntilTheFlowerUnlock() {
    assertEquals(0, MatchClock.nectarAllowance(0, false));
    assertEquals(1, MatchClock.nectarAllowance(1, false));
    assertEquals(3, MatchClock.nectarAllowance(3, false));
    assertEquals(
        MatchClock.STAGED_NECTAR, MatchClock.nectarAllowance(MatchClock.STAGED_NECTAR, false));
  }

  @Test
  public void testNectarAllowanceIsCappedAtTheStagedFive() {
    assertEquals(5, MatchClock.STAGED_NECTAR);
    assertEquals(5, MatchClock.nectarAllowance(6, false));
    assertEquals(5, MatchClock.nectarAllowance(500, false));
    assertEquals(5, MatchClock.nectarAllowance(Integer.MAX_VALUE, false));
  }

  @Test
  public void testNectarAllowanceNeverNegativeForBadTipCounts() {
    assertEquals(0, MatchClock.nectarAllowance(-1, false));
    assertEquals(0, MatchClock.nectarAllowance(Integer.MIN_VALUE, false));
  }

  @Test
  public void testUnlockGivesEveryNectarRegardlessOfTips() {
    for (int tips : new int[] {-3, 0, 1, 4, 9}) {
      assertEquals(5, MatchClock.nectarAllowance(tips, true), "tips=" + tips);
    }
  }

  @Test
  public void testNectarAllowanceFollowsTheClockUnlock() {
    MatchClock clock = teleop();
    clock.start();
    assertEquals(1, MatchClock.nectarAllowance(1, clock.isFlowerUnlocked()));
    now += TELEOP_MS - UNLOCK_REMAINING;
    assertEquals(5, MatchClock.nectarAllowance(1, clock.isFlowerUnlocked()));
  }

  @Test
  public void testFromConfigUsesTheConfiguredPeriodLengths() {
    assertEquals(30_000, config.match.auto_ms);
    assertEquals(120_000, config.match.teleop_ms);

    MatchClock a = MatchClock.fromConfig(MatchClock.Period.AUTO, () -> now);
    MatchClock t = MatchClock.fromConfig(MatchClock.Period.TELEOP, () -> now);
    assertEquals(MatchClock.Period.AUTO, a.period());
    assertEquals(MatchClock.Period.TELEOP, t.period());
    assertEquals(config.match.auto_ms, a.remainingMs());
    assertEquals(config.match.teleop_ms, t.remainingMs());

    t.start();
    now += config.match.teleop_ms - config.match.flower_unlock_remaining_ms;
    assertTrue(t.isFlowerUnlocked());
    now -= 1;
    assertFalse(t.isFlowerUnlocked());
  }

  @Test
  @Tag("push-config")
  public void testConfiguredWindowsFitInsideTheirPeriods() {
    assertTrue(config.match.flower_unlock_remaining_ms < config.match.teleop_ms);
    assertTrue(config.match.final_warning_remaining_ms < config.match.flower_unlock_remaining_ms);
    assertTrue(
        config.auto.park_reserve_ms > 0 && config.auto.park_reserve_ms < config.match.auto_ms,
        "park reserve must fit inside AUTO");
  }
}
