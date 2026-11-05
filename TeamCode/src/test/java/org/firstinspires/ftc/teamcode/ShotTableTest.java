package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

public class ShotTableTest {

  /** distance, hood servo, rpm */
  private static final double[] POINTS = {
    48.0, 0.100, 900.0,
    72.0, 0.200, 1000.0,
    96.0, 0.300, 1100.0,
    120.0, 0.400, 1200.0
  };

  private static ShotTable table() {
    return ShotTable.fromPoints(POINTS);
  }

  @Test
  public void testExactRowsReturnedUnchanged() {
    ShotTable t = table();
    for (int i = 0; i < POINTS.length; i += 3) {
      ShotTable.Shot s = t.lookup(POINTS[i]);
      assertEquals(
          POINTS[i + 1], s.hoodServoPosition(), 1e-9, "hood at calibrated row " + POINTS[i]);
      assertEquals(POINTS[i + 2], s.rpm(), 1e-9, "rpm at calibrated row " + POINTS[i]);
      assertTrue(s.withinCalibratedRange(), "a calibrated row is in range");
    }
  }

  @Test
  public void testLinearInterpolationBetweenRows() {
    ShotTable t = table();
    ShotTable.Shot mid = t.lookup(60.0); // halfway between 48 and 72
    assertEquals(0.150, mid.hoodServoPosition(), 1e-9);
    assertEquals(950.0, mid.rpm(), 1e-9);
    assertTrue(mid.withinCalibratedRange());

    ShotTable.Shot quarter = t.lookup(78.0); // 25% from 72 toward 96
    assertEquals(0.225, quarter.hoodServoPosition(), 1e-9);
    assertEquals(1025.0, quarter.rpm(), 1e-9);
  }

  /**
   * Outside the measured range the table clamps and flags rather than extrapolating. Extrapolation
   * is exactly the guess this class exists to avoid: nothing was ever observed out there.
   */
  @Test
  public void testOutsideCalibratedRangeClampsAndFlags() {
    ShotTable t = table();

    ShotTable.Shot tooClose = t.lookup(20.0);
    assertEquals(0.100, tooClose.hoodServoPosition(), 1e-9, "clamps to the nearest row");
    assertEquals(900.0, tooClose.rpm(), 1e-9);
    assertFalse(tooClose.withinCalibratedRange(), "below the table must not read as calibrated");

    ShotTable.Shot tooFar = t.lookup(200.0);
    assertEquals(0.400, tooFar.hoodServoPosition(), 1e-9);
    assertEquals(1200.0, tooFar.rpm(), 1e-9);
    assertFalse(tooFar.withinCalibratedRange(), "beyond the table must not read as calibrated");
  }

  @Test
  public void testInterpolationIsMonotonicWhenTableIs() {
    ShotTable t = table();
    double prevHood = -1;
    double prevRpm = -1;
    for (double d = 48.0; d <= 120.0; d += 1.0) {
      ShotTable.Shot s = t.lookup(d);
      assertTrue(
          s.hoodServoPosition() >= prevHood, "hood must not decrease across a monotonic table");
      assertTrue(s.rpm() >= prevRpm, "rpm must not decrease across a monotonic table");
      prevHood = s.hoodServoPosition();
      prevRpm = s.rpm();
    }
  }

  @Test
  public void testMalformedTablesAreRejectedLoudly() {
    // A silently mis-parsed table would command arbitrary hood and flywheel values every shot.
    assertThrows(IllegalStateException.class, () -> ShotTable.fromPoints(null));
    assertThrows(IllegalStateException.class, () -> ShotTable.fromPoints(new double[] {}));
    assertThrows(
        IllegalStateException.class,
        () -> ShotTable.fromPoints(new double[] {48.0, 0.1, 900.0, 72.0}),
        "length not a multiple of 3");
    assertThrows(
        IllegalStateException.class,
        () -> ShotTable.fromPoints(new double[] {72.0, 0.2, 1000.0, 48.0, 0.1, 900.0}),
        "distances must strictly increase");
    assertThrows(
        IllegalStateException.class,
        () -> ShotTable.fromPoints(new double[] {48.0, 0.1, 900.0, 48.0, 0.2, 1000.0}),
        "duplicate distances would divide by zero when interpolating");
  }

  @Test
  public void testSinglePointTableIsUsableAndAlwaysClamps() {
    ShotTable t = ShotTable.fromPoints(new double[] {72.0, 0.25, 1050.0});
    assertEquals(1, t.size());
    assertEquals(0.25, t.lookup(72.0).hoodServoPosition(), 1e-9);
    assertEquals(0.25, t.lookup(30.0).hoodServoPosition(), 1e-9);
    assertEquals(1050.0, t.lookup(200.0).rpm(), 1e-9);
    assertFalse(t.lookup(200.0).withinCalibratedRange());
  }

  @Test
  public void testRangeAccessors() {
    ShotTable t = table();
    assertEquals(48.0, t.minDistance(), 1e-9);
    assertEquals(120.0, t.maxDistance(), 1e-9);
    assertEquals(4, t.size());
  }

  @Test
  public void testFromPointsIsCalibratedUnlessToldOtherwise() {
    assertTrue(table().isCalibrated(), "a table built from measured rows is calibrated");
    assertTrue(ShotTable.fromPoints(POINTS, true).isCalibrated());
    assertFalse(ShotTable.fromPoints(POINTS, false).isCalibrated());
  }

  @Test
  public void testCalibrationFlagDoesNotChangeTheLookup() {
    ShotTable placeholder = ShotTable.fromPoints(POINTS, false);
    ShotTable measured = table();
    for (double d : new double[] {30.0, 48.0, 60.0, 100.0, 130.0}) {
      assertEquals(
          measured.lookup(d).hoodServoPosition(), placeholder.lookup(d).hoodServoPosition(), 0.0);
      assertEquals(measured.lookup(d).rpm(), placeholder.lookup(d).rpm(), 0.0);
      assertEquals(
          measured.lookup(d).withinCalibratedRange(),
          placeholder.lookup(d).withinCalibratedRange());
    }
  }

  @Test
  public void testNonFiniteValuesAreRejected() {
    for (double bad :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      assertThrows(
          IllegalStateException.class,
          () -> ShotTable.fromPoints(new double[] {bad, 0.1, 900.0}),
          "distance " + bad);
      assertThrows(
          IllegalStateException.class,
          () -> ShotTable.fromPoints(new double[] {48.0, bad, 900.0}),
          "hood " + bad);
      assertThrows(
          IllegalStateException.class,
          () -> ShotTable.fromPoints(new double[] {48.0, 0.1, bad}),
          "rpm " + bad);
    }
  }

  @Test
  public void testNonFiniteLookupNeverReturnsACalibratedShot() {
    ShotTable table = table();
    for (double bad :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      ShotTable.Shot shot = table.lookup(bad);
      assertFalse(shot.withinCalibratedRange());
      assertTrue(Double.isFinite(shot.hoodServoPosition()));
      assertTrue(Double.isFinite(shot.rpm()));
    }
  }

  @Test
  public void testRangeEdgesCountAsCalibrated() {
    ShotTable t = table();
    assertTrue(t.lookup(48.0).withinCalibratedRange());
    assertTrue(t.lookup(120.0).withinCalibratedRange());
    assertFalse(t.lookup(47.9).withinCalibratedRange());
    assertFalse(t.lookup(120.1).withinCalibratedRange());
    assertTrue(
        t.lookup(47.9).reason().contains("below the calibrated range"),
        "the refusal explains itself");
    assertTrue(t.lookup(120.1).reason().contains("beyond the calibrated range"));
    assertEquals("Valid", t.lookup(60.0).reason());
  }

  /**
   * Both SCORING ELEMENTS ship with a usable table, and the nectar rows are separate from pollen.
   */
  @Test
  public void testShippedTablesLoadForEveryBallType() {
    for (BallType type : BallType.values()) {
      ShotTable t = ShotTable.fromConfig(type);
      assertTrue(t.size() >= 2, type + " table has rows");
      assertEquals(
          config.shooter.shot_table.calibrated,
          t.isCalibrated(),
          type + " table honours shooter.shot_table.calibrated");
      assertTrue(t.minDistance() < t.maxDistance());
      // The solver's distance gate must sit inside what the table measured, or its upper and lower
      // ends would be refused by the table rather than by the gate that documents them.
      assertTrue(
          t.minDistance() <= config.shooter.ballistics.min_valid_distance + 1e-9,
          type + " table must cover min_valid_distance");
      assertTrue(
          t.maxDistance() >= config.shooter.ballistics.max_valid_distance - 1e-9,
          type + " table must cover max_valid_distance");
    }

    ShotTable pollen = ShotTable.fromConfig(BallType.POLLEN);
    ShotTable nectar = ShotTable.fromConfig(BallType.NECTAR);
    double d = 48.0;
    assertTrue(
        pollen.lookup(d).rpm() != nectar.lookup(d).rpm()
            || pollen.lookup(d).hoodServoPosition() != nectar.lookup(d).hoodServoPosition(),
        "NECTAR (3.6 in, heavier) is not shot with the POLLEN rows");
  }

  @Test
  public void testShippedRowsAreOrderedAndUseLegalServoPositions() {
    for (BallType type : BallType.values()) {
      ShotTable t = ShotTable.fromConfig(type);
      double previousRpm = 0.0;
      for (double d = t.minDistance(); d <= t.maxDistance(); d += 1.0) {
        ShotTable.Shot s = t.lookup(d);
        assertTrue(
            s.hoodServoPosition() >= 0.0 && s.hoodServoPosition() <= 1.0, type + " hood at " + d);
        assertTrue(s.rpm() > 0.0 && s.rpm() <= config.shooter.max_rpm, type + " rpm at " + d);
        assertTrue(
            s.rpm() >= previousRpm - 1e-9, type + " rpm must not fall with distance at " + d);
        previousRpm = s.rpm();
      }
    }
  }
}
