package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.ballistics.ShotTimeTable;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

/**
 * The autonomous shoot-window table is keyed by distance, in exactly the format the Shot Timing
 * Tuner exports. (DECODE's runtime read RPM-keyed rows from a different key than the tuner wrote.)
 */
public class ShotTimeTableTest {

  @Test
  public void interpolatesByDistanceAndClampsAtTheEnds() {
    ShotTimeTable table = ShotTimeTable.fromPoints(new double[] {24.0, 2000, 48.0, 3000});
    assertNotNull(table);
    assertEquals(2, table.size());
    assertEquals(24.0, table.minDistance(), 0.0);
    assertEquals(48.0, table.maxDistance(), 0.0);
    assertEquals(2500, table.lookupMs(36.0));
    assertEquals(2000, table.lookupMs(10.0));
    assertEquals(3000, table.lookupMs(90.0));
    assertEquals(2000, table.lookupMs(Double.NaN));
  }

  @Test
  public void emptyTableMeansUseTheFallback() {
    assertNull(ShotTimeTable.fromPoints(new double[0]));
    assertNull(ShotTimeTable.fromPoints(null));
  }

  @Test
  public void malformedTablesAreRejected() {
    assertThrows(IllegalStateException.class, () -> ShotTimeTable.fromPoints(new double[] {24.0}));
    assertThrows(
        IllegalStateException.class,
        () -> ShotTimeTable.fromPoints(new double[] {48.0, 2000, 24.0, 3000}));
    assertThrows(
        IllegalStateException.class,
        () -> ShotTimeTable.fromPoints(new double[] {24.0, -5, 48.0, 3000}));
    assertThrows(
        IllegalStateException.class,
        () -> ShotTimeTable.fromPoints(new double[] {Double.NaN, 2000, 48.0, 3000}));
  }

  @Test
  public void shippedTableParsesAndEveryWindowIsPositive() {
    ShotTimeTable table = ShotTimeTable.fromConfig();
    assertNotNull(table, "auto.shot_time_points must be configured");
    for (double d = 0.0; d <= 100.0; d += 6.0) {
      assertTrue(ShotTimeTable.windowMsFor(d) > 0);
    }
    assertTrue(config.auto.shoot_wait_ms > 0);
  }
}
