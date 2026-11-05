package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.ConfigSanity;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** The INIT config report: silent for the shipped config, and loud for each broken combination. */
public class ConfigSanityTest {

  @Test
  @Tag("push-config")
  public void theShippedConfigHasNoProblems() {
    assertEquals(List.of(), ConfigSanity.problems());
  }

  private static void assertReported(String key) {
    List<String> problems = ConfigSanity.problems();
    assertTrue(
        problems.stream().anyMatch(p -> p.contains(key)),
        () -> key + " not reported in " + problems);
  }

  @Test
  public void anInvertedTurretToleranceIsReported() {
    config.turret.tolerance.max_deg = config.turret.tolerance.min_deg / 2;
    assertReported("turret.tolerance");
  }

  @Test
  public void aStallThresholdTheTurretCanNeverReachIsReported() {
    config.turret.stall.power_threshold = config.turret.max_power_output;
    assertReported("turret.stall.power_threshold");
  }

  @Test
  public void aZeroVoltageThatPutsTheWrapInTheTravelIsReported() {
    var enc = config.turret.analog_encoder;
    enc.zero_voltage = (enc.min_voltage + enc.max_voltage) / 2 + enc.full_scale_voltage / 2;
    assertReported("turret.analog_encoder");
  }

  @Test
  public void aPreferredShotSpeedAboveTheMaximumIsReported() {
    config.shooter.ballistics.preferred_shot_rpm = config.shooter.ballistics.max_shot_rpm + 1;
    assertReported("shooter.ballistics");
  }

  @Test
  public void aFeedWindowThatExcludesTheTargetIsReported() {
    config.shooter.min_transfer_threshold = 1.0;
    assertReported("shooter feed window");
  }

  @Test
  public void aHardStopBeyondTheSlowDownIsReported() {
    config.casablanca.side.hard_stop = config.casablanca.side.slow_down;
    assertReported("casablanca.side");
  }

  @Test
  public void aHiveThresholdOutsideTheCellHeightsIsReported() {
    config.hive.up_height_threshold_in = config.field.hive.up_opening_height_in + 1;
    assertReported("hive.up_height_threshold_in");
  }

  @Test
  public void aParkReserveAsLongAsAutoIsReported() {
    config.auto.park_reserve_ms = config.match.auto_ms;
    assertReported("auto.park_reserve_ms");
  }

  @Test
  public void aVolleyLargerThanTheInventoryIsReported() {
    config.inventory.capacity_count = 2;
    assertReported("auto.balls_per_volley_count");
  }

  /** A malformed table is reported at INIT, not thrown out of the report and through init. */
  @Test
  public void aMalformedShotTableIsReportedNotThrown() {
    config.shooter.shot_table.pollen_points = new double[] {24, 0.5};
    assertReported("shot table POLLEN");
  }

  /** A measured row the runtime speed band would silently change is named. */
  @Test
  public void aCalibratedRowBelowThePreferredSpeedIsReported() {
    var t = config.shooter.shot_table;
    t.calibrated = true;
    double below = config.shooter.ballistics.preferred_shot_rpm - 100;
    t.pollen_points = new double[] {24, 0.5, below, 48, 0.4, below + 300};
    assertReported("row at 24.0 in");
  }
}
