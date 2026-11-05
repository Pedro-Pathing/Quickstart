package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;
import org.junit.jupiter.api.Test;

/** The G407 four-element cap: counting in, counting out, and not lying when it cannot know. */
public class BallInventoryTest {

  private static final long DEBOUNCE = 40;

  /** Holds the sensor blocked from {@code from} to {@code to} inclusive in 10 ms steps. */
  private static boolean holdBlocked(BallInventory inv, long from, long to, boolean outward) {
    boolean counted = false;
    for (long t = from; t <= to; t += 10) {
      counted |= inv.updateEntrySensor(true, t, DEBOUNCE, outward);
    }
    return counted;
  }

  @Test
  public void testStartsEmptyAndKnown() {
    BallInventory inv = new BallInventory(4, true);

    assertEquals(4, inv.capacity());
    assertTrue(inv.isSensed());
    assertTrue(inv.isKnown());
    assertEquals(0, inv.count());
    assertTrue(inv.isEmpty());
    assertFalse(inv.isFull());
  }

  @Test
  public void testCapacityMustBeAtLeastOne() {
    assertThrows(IllegalArgumentException.class, () -> new BallInventory(0, true));
    assertThrows(IllegalArgumentException.class, () -> new BallInventory(-4, false));
    assertEquals(1, new BallInventory(1, true).capacity());
  }

  @Test
  public void testSetClampsToZeroAndOneOverCapacity() {
    BallInventory inv = new BallInventory(4, false);

    inv.set(3);
    assertEquals(3, inv.count());
    assertFalse(inv.isFull());
    assertFalse(inv.isEmpty());

    inv.set(4);
    assertTrue(inv.isFull(), "four is the G407 limit");

    inv.set(5);
    assertEquals(5, inv.count(), "one over capacity is representable: a fifth ball got in");
    assertTrue(inv.isFull());

    inv.set(99);
    assertEquals(5, inv.count());

    inv.set(-3);
    assertEquals(0, inv.count());
    assertTrue(inv.isEmpty());
  }

  @Test
  public void testPreloadCountFromConfigFitsTheCapacity() {
    BallInventory inv =
        new BallInventory(config.inventory.capacity_count, config.inventory.entry_sensor_enabled);
    inv.set(config.inventory.preload_count);
    assertEquals(config.inventory.preload_count, inv.count());
    assertTrue(
        config.inventory.preload_count <= config.inventory.capacity_count,
        "four pre-loads is the legal maximum");
    assertEquals(4, config.inventory.capacity_count);
  }

  @Test
  public void testOnShotCountsDownAndStopsAtZero() {
    BallInventory inv = new BallInventory(4, false);
    inv.set(2);

    inv.onShot();
    assertEquals(1, inv.count());
    inv.onShot();
    assertEquals(0, inv.count());
    assertTrue(inv.isEmpty());
    inv.onShot();
    inv.onShot();
    assertEquals(0, inv.count(), "a miscounted extra shot cannot go below empty");
  }

  // -------------------------------------------------------------------------------------------
  // Unknown counts

  @Test
  public void testIntakeWithoutASensorMakesTheCountUnknown() {
    BallInventory inv = new BallInventory(4, false);
    inv.set(4);
    assertTrue(inv.isFull());

    inv.markUnknown();

    assertFalse(inv.isKnown());
    assertFalse(inv.isFull(), "an unknown count never claims full, so the intake keeps working");
    assertFalse(inv.isEmpty(), "and never claims empty, so a volley is not cut short");
    assertEquals(4, inv.count(), "the last known value is still reported");
  }

  @Test
  public void testSettingACountMakesItKnownAgain() {
    BallInventory inv = new BallInventory(4, false);
    inv.markUnknown();
    assertFalse(inv.isKnown());

    inv.set(4);

    assertTrue(inv.isKnown());
    assertTrue(inv.isFull());
  }

  @Test
  public void testASensedInventoryStaysKnownWhenTheIntakeRuns() {
    BallInventory inv = new BallInventory(4, true);
    inv.set(2);

    inv.markUnknown();

    assertTrue(inv.isKnown());
    assertEquals(2, inv.count());
  }

  @Test
  public void testShotsStillCountDownWhileUnknown() {
    BallInventory inv = new BallInventory(4, false);
    inv.set(4);
    inv.markUnknown();
    inv.onShot();
    assertEquals(3, inv.count());
    assertFalse(inv.isKnown());
  }

  // -------------------------------------------------------------------------------------------
  // Entry sensor

  @Test
  public void testAnElementIsCountedOnceItHasBlockedTheSensorForTheDebounceTime() {
    BallInventory inv = new BallInventory(4, true);

    assertFalse(inv.updateEntrySensor(true, 1000, DEBOUNCE));
    assertFalse(inv.updateEntrySensor(true, 1039, DEBOUNCE));
    assertEquals(0, inv.count());

    assertTrue(inv.updateEntrySensor(true, 1040, DEBOUNCE), "exactly the debounce time");
    assertEquals(1, inv.count());
  }

  @Test
  public void testOneElementIsCountedOnceHoweverLongItSitsInTheSensor() {
    BallInventory inv = new BallInventory(4, true);

    assertTrue(holdBlocked(inv, 0, 1000, false));
    assertEquals(1, inv.count());
    assertFalse(holdBlocked(inv, 1010, 5000, false));
    assertEquals(1, inv.count(), "still one element");
  }

  @Test
  public void testTheSensorMustClearBeforeTheNextElementCounts() {
    BallInventory inv = new BallInventory(4, true);

    holdBlocked(inv, 0, 100, false);
    inv.updateEntrySensor(false, 110, DEBOUNCE);
    holdBlocked(inv, 120, 220, false);
    inv.updateEntrySensor(false, 230, DEBOUNCE);
    holdBlocked(inv, 240, 340, false);

    assertEquals(3, inv.count());
  }

  @Test
  public void testABlipShorterThanTheDebounceIsNotAnElement() {
    BallInventory inv = new BallInventory(4, true);

    // Blocked for 30 ms, then clear: noise or a ball bouncing past.
    inv.updateEntrySensor(true, 0, DEBOUNCE);
    inv.updateEntrySensor(true, 30, DEBOUNCE);
    inv.updateEntrySensor(false, 31, DEBOUNCE);
    assertEquals(0, inv.count());

    // The debounce restarts from the next block, it does not remember the first blip.
    assertFalse(inv.updateEntrySensor(true, 50, DEBOUNCE));
    assertFalse(inv.updateEntrySensor(true, 89, DEBOUNCE));
    assertTrue(inv.updateEntrySensor(true, 90, DEBOUNCE));
    assertEquals(1, inv.count());
  }

  @Test
  public void testZeroDebounceCountsOnTheFirstBlockedReading() {
    BallInventory inv = new BallInventory(4, true);
    assertTrue(inv.updateEntrySensor(true, 5, 0));
    assertEquals(1, inv.count());
    assertFalse(inv.updateEntrySensor(true, 6, 0));
  }

  @Test
  public void testFillingTheMagazineReportsFullAtFour() {
    BallInventory inv = new BallInventory(4, true);

    for (int i = 0; i < 4; i++) {
      assertFalse(inv.isFull(), "not full before element " + (i + 1));
      long t = i * 1000L;
      holdBlocked(inv, t, t + 100, false);
      inv.updateEntrySensor(false, t + 200, DEBOUNCE);
    }

    assertTrue(inv.isFull());
    assertEquals(4, inv.count());
  }

  @Test
  public void testAFifthElementIsCountedOneOverTheCapAndNoFurther() {
    BallInventory inv = new BallInventory(4, true);
    inv.set(4);

    holdBlocked(inv, 0, 100, false);
    assertEquals(5, inv.count(), "the fifth is counted so the driver can see the violation");
    inv.updateEntrySensor(false, 200, DEBOUNCE);
    holdBlocked(inv, 300, 400, false);
    assertEquals(5, inv.count(), "but the count saturates");
    assertTrue(inv.isFull());
  }

  @Test
  public void testEjectingRemovesAnElementAndStopsAtZero() {
    BallInventory inv = new BallInventory(4, true);
    inv.set(1);

    assertTrue(holdBlocked(inv, 0, 100, true));
    assertEquals(0, inv.count());
    inv.updateEntrySensor(false, 200, DEBOUNCE);

    assertTrue(holdBlocked(inv, 300, 400, true), "a pass is still reported");
    assertEquals(0, inv.count(), "an eject cannot go below empty");
  }

  @Test
  public void testThreeArgumentOverloadCountsInward() {
    BallInventory inv = new BallInventory(4, true);
    inv.updateEntrySensor(true, 0, DEBOUNCE);
    assertTrue(inv.updateEntrySensor(true, 40, DEBOUNCE));
    assertEquals(1, inv.count());
  }

  @Test
  public void testUnknownInventoryDoesNotInventACountFromTheSensor() {
    BallInventory inv = new BallInventory(4, false);
    inv.markUnknown();
    inv.set(3);
    inv.markUnknown();

    assertTrue(holdBlocked(inv, 0, 100, false), "the pass is detected");
    assertEquals(3, inv.count(), "but an unknown count is not advanced");
    assertFalse(inv.isKnown());
  }

  @Test
  public void testUnsensedInventoryStillTracksTheDebounceStateConsistently() {
    // Same state machine whether or not the config says a sensor is fitted.
    BallInventory inv = new BallInventory(4, false);
    holdBlocked(inv, 0, 100, false);
    assertEquals(1, inv.count());
  }
}
