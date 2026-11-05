package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The feed gate truth table and latch hysteresis; {@link FeedLaw}'s contracts are OpenJML-proven.
 */
class FeedLawTest {

  @Test
  void gateTruthTableForEveryInput() {
    boolean[] b = {false, true};
    for (boolean fly : b)
      for (boolean turret : b)
        for (boolean require : b)
          for (boolean current : b)
            for (boolean valid : b)
              for (boolean settled : b) {
                boolean expected = fly && turret && (!require || (current && valid && settled));
                assertEquals(expected, FeedLaw.gate(fly, turret, require, current, valid, settled));
              }
  }

  @Test
  void waivedShotStillNeedsFlywheelAndTurret() {
    assertTrue(FeedLaw.gate(true, true, false, false, false, false));
    assertFalse(FeedLaw.gate(false, true, false, true, true, true));
    assertFalse(FeedLaw.gate(true, false, false, true, true, true));
  }

  @Test
  void staleSolutionNeverFeedsWhenRequired() {
    assertFalse(FeedLaw.gate(true, true, true, false, true, true));
    assertFalse(FeedLaw.gate(true, true, true, true, true, false));
  }

  @Test
  void isCurrentNeedsEpochAndBallType() {
    assertTrue(FeedLaw.isCurrent(3, 3, 1, 1));
    assertFalse(FeedLaw.isCurrent(2, 3, 1, 1));
    assertFalse(FeedLaw.isCurrent(3, 3, 0, 1));
  }

  @Test
  void hysteresis() {
    assertTrue(FeedLaw.shouldFeed(0.95, false, 0.95, 1.05, 0.8));
    assertTrue(FeedLaw.shouldFeed(1.05, false, 0.95, 1.05, 0.8));
    assertFalse(FeedLaw.shouldFeed(1.06, false, 0.95, 1.05, 0.8));
    assertFalse(FeedLaw.shouldFeed(0.9, false, 0.95, 1.05, 0.8));
    assertTrue(FeedLaw.shouldFeed(0.8, true, 0.95, 1.05, 0.8));
    assertTrue(FeedLaw.shouldFeed(1.2, true, 0.95, 1.05, 0.8));
    assertFalse(FeedLaw.shouldFeed(0.79, true, 0.95, 1.05, 0.8));
    assertFalse(FeedLaw.shouldFeed(Double.NaN, false, 0.95, 1.05, 0.8));
    assertFalse(FeedLaw.shouldFeed(Double.NaN, true, 0.95, 1.05, 0.8));
  }

  @Test
  void ratioAndBandAreSafeForBadTargets() {
    assertEquals(0.0, FeedLaw.ratio(100, 0));
    assertEquals(0.0, FeedLaw.ratio(100, -5));
    assertEquals(0.0, FeedLaw.ratio(100, Double.NaN));
    assertEquals(0.5, FeedLaw.ratio(50, 100));
    assertTrue(FeedLaw.inBand(100, 100, 0.95, 1.05));
    assertFalse(FeedLaw.inBand(106, 100, 0.95, 1.05));
    assertFalse(FeedLaw.inBand(Double.NaN, 100, 0.95, 1.05));
  }
}
