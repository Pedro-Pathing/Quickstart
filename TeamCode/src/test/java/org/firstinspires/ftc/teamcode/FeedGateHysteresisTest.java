package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.junit.jupiter.api.Test;

/**
 * Covers the latching feed gate in {@link ShotController#shouldFeed}. A ring entering the flywheel
 * drags its speed down, so a gate with a single hard threshold cuts the intake mid-feed, lets the
 * wheel recover, and re-opens — the intake visibly stutters instead of running continuously. These
 * pin the hysteresis that prevents that.
 */
public class FeedGateHysteresisTest {

  // Live config.yaml values.
  private static final double ARM_LOW = 0.95;
  private static final double ARM_HIGH = 1.05;
  private static final double RELEASE_LOW = 0.80;

  private static boolean feed(double ratio, boolean feeding) {
    return ShotController.shouldFeed(ratio, feeding, ARM_LOW, ARM_HIGH, RELEASE_LOW);
  }

  @Test
  public void testGateStaysShutDuringSpinUp() {
    assertFalse(feed(0.50, false), "well below target must not feed");
    assertFalse(feed(0.94, false), "just under the arm floor must not feed");
  }

  @Test
  public void testGateArmsInsideWindow() {
    assertTrue(feed(ARM_LOW, false), "at the arm floor");
    assertTrue(feed(1.00, false), "on target");
    assertTrue(feed(ARM_HIGH, false), "at the arm ceiling");
  }

  @Test
  public void testOverspeedBlocksArming() {
    assertFalse(feed(1.06, false), "above the arm ceiling must not start a feed");
  }

  @Test
  public void testFeedSurvivesRingSag() {
    // The whole point: a ring pulls the wheel under the arm floor, and the feed must continue.
    assertTrue(feed(0.90, true), "sag below arm floor keeps feeding");
    assertTrue(feed(0.81, true), "deeper sag still keeps feeding");
    assertTrue(feed(RELEASE_LOW, true), "exactly at the release floor keeps feeding");
  }

  @Test
  public void testFeedReleasesWhenFlywheelTrulyStalls() {
    assertFalse(feed(0.79, true), "below the release floor cuts the feed");
    assertFalse(feed(0.10, true), "stalled wheel cuts the feed");
  }

  @Test
  public void testOverspeedDoesNotInterruptRunningFeed() {
    // Rings only ever slow the wheel, so an over-speed reading mid-feed is the controller
    // overshooting on recovery. Cutting the feed for it would stutter from the other direction.
    assertTrue(feed(1.20, true), "overshoot while feeding must not cut the feed");
  }

  @Test
  public void testReleaseFloorIsBelowArmFloorSoBandIsRealHysteresis() {
    assertTrue(
        RELEASE_LOW < ARM_LOW,
        "release floor must sit below the arm floor or there is no hysteresis at all");

    // A ratio inside the band resolves purely on prior state — that is the hysteresis.
    double insideBand = 0.90;
    assertFalse(feed(insideBand, false), "inside the band, a shut gate stays shut");
    assertTrue(feed(insideBand, true), "inside the band, an open gate stays open");
  }
}
