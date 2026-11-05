package org.firstinspires.ftc.teamcode.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.ballistics.ShotTimeTable;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Tuning values that are each in range but do not work together, reported at INIT instead of being
 * discovered mid-session. The robot code stays safe with any of these (TuningFuzzProperties checks
 * that); this is about a subsystem silently doing nothing useful, e.g. a stall watchdog that can
 * never arm or a shot table the solver refuses. Each message says what the driver would see.
 */
public final class ConfigSanity {
  private ConfigSanity() {}

  /** Every problem with the current config, worded for the Driver Station; empty if none. */
  public static List<String> problems() {
    List<String> p = new ArrayList<>();
    turret(p);
    shooter(p);
    drive(p);
    game(p);
    return p;
  }

  private static void turret(List<String> p) {
    var t = config.turret;
    if (t.travel.min_angle >= t.travel.max_angle) {
      p.add(
          fmt(
              "turret.travel: min_angle %.1f is not below max_angle %.1f",
              t.travel.min_angle, t.travel.max_angle));
    }
    if (t.tolerance.min_deg > t.tolerance.max_deg) {
      p.add(
          fmt(
              "turret.tolerance: min_deg %.2f > max_deg %.2f (aim tolerance pinned at the smaller)",
              t.tolerance.min_deg, t.tolerance.max_deg));
    }
    if (t.analog_encoder.enabled) {
      String encoder = Turret.encoderRangeWarning(t.analog_encoder.zero_voltage);
      if (encoder != null) {
        p.add("turret.analog_encoder: " + encoder);
      }
    }
    double maxPower = t.max_power_output;
    if (maxPower <= 0) {
      p.add("turret.max_power_output is 0: the turret will never move");
    }
    if (Math.max(t.ks_positive, t.ks_negative) >= maxPower && maxPower > 0) {
      p.add(
          fmt(
              "turret ks_positive/ks_negative (%.3f/%.3f) reach max_power_output %.2f: the turret cannot make fine corrections",
              t.ks_positive, t.ks_negative, maxPower));
    }
    if (t.stall.enabled && t.stall.power_threshold >= maxPower) {
      p.add(
          fmt(
              "turret.stall.power_threshold %.2f >= max_power_output %.2f: the stall watchdog can never arm",
              t.stall.power_threshold, maxPower));
    }
    if (t.integral.limit_power > maxPower && maxPower > 0) {
      p.add(
          fmt(
              "turret.integral.limit_power %.2f exceeds max_power_output %.2f",
              t.integral.limit_power, maxPower));
    }
  }

  private static void shooter(List<String> p) {
    var s = config.shooter;
    var b = s.ballistics;
    if (s.max_rpm <= 0) {
      // No early return: the checks below only compare values, and stopping here hid every other
      // shooter problem (e.g. a ballistics config the solver refuses) from the INIT report.
      p.add("shooter.max_rpm must be positive: the flywheel cannot be commanded");
    }
    if (s.use_ftc_pid) {
      double recipeP = 0.1 * s.motor_pidf.f;
      if (s.motor_pidf.p > 3 * recipeP) {
        p.add(
            fmt(
                "shooter.motor_pidf.p %.1f is %.0fx REV's recipe (0.1 F = %.2f): expect the flywheel to oscillate",
                s.motor_pidf.p, s.motor_pidf.p / recipeP, recipeP));
      }
    } else if (!(s.pidf.i > 0)) {
      p.add(
          "shooter.pidf.i is 0: with P alone any kS/kV error keeps the wheel outside the feed band");
    }
    if (s.constant_rpm > s.max_rpm) {
      p.add(
          fmt(
              "shooter.constant_rpm %.0f > max_rpm %.0f: fixed shots are capped at max_rpm",
              s.constant_rpm, s.max_rpm));
    }
    if (b.max_shot_rpm > s.max_rpm) {
      p.add(
          fmt(
              "shooter.ballistics.max_shot_rpm %.0f > max_rpm %.0f: solved shots above max_rpm are capped",
              b.max_shot_rpm, s.max_rpm));
    }
    if (s.min_transfer_threshold >= 1.0 || s.max_velocity_threshold <= 1.0) {
      p.add(
          fmt(
              "shooter feed window [%.2f, %.2f] of target speed must contain 1.0, or the flywheel is never ready",
              s.min_transfer_threshold, s.max_velocity_threshold));
    }
    if (s.feed_release_threshold > s.min_transfer_threshold) {
      p.add(
          fmt(
              "shooter.feed_release_threshold %.2f > min_transfer_threshold %.2f: feeding stops as soon as it starts",
              s.feed_release_threshold, s.min_transfer_threshold));
    }
    if (b.min_valid_distance >= b.max_valid_distance) {
      p.add(
          fmt(
              "shooter.ballistics: min_valid_distance %.0f >= max_valid_distance %.0f: every shot is out of range",
              b.min_valid_distance, b.max_valid_distance));
    }
    try {
      BallisticsParameters.fromConfig();
    } catch (RuntimeException e) {
      p.add("shooter.ballistics: " + e.getMessage() + " (every shot is refused)");
    }
    for (BallType type : BallType.values()) {
      try {
        ShotTable.fromConfig(type);
      } catch (RuntimeException e) {
        p.add("shot table " + type + ": " + e.getMessage() + " (" + type + " shots are refused)");
        continue;
      }
      rowsOutsideTheSpeedBand(p, type);
    }
    try {
      ShotTimeTable.fromConfig();
    } catch (RuntimeException e) {
      p.add("auto.shot_time_points: " + e.getMessage());
    }
  }

  /**
   * Solved speeds are held inside [preferred_shot_rpm, max_shot_rpm] at runtime. A calibrated row
   * outside that band would be fired at a speed nobody measured, so name each one.
   */
  private static void rowsOutsideTheSpeedBand(List<String> p, BallType type) {
    var t = config.shooter.shot_table;
    if (!t.calibrated) {
      return;
    }
    double lo = config.shooter.ballistics.preferred_shot_rpm;
    double hi = config.shooter.ballistics.max_shot_rpm;
    double[] rows = type == BallType.NECTAR ? t.nectar_points : t.pollen_points;
    for (int i = 0; i + 2 < rows.length; i += 3) {
      double rpm = rows[i + 2];
      if (rpm < lo || rpm > hi) {
        p.add(
            fmt(
                "shot table %s row at %.1f in is %.0f ticks/s but fires at %.0f (band preferred_shot_rpm %.0f..max_shot_rpm %.0f): widen the band",
                type, rows[i], rpm, Math.clamp(rpm, Math.min(lo, hi), Math.max(lo, hi)), lo, hi));
      }
    }
  }

  private static void drive(List<String> p) {
    var c = config.casablanca;
    if (c.depth.hard_stop >= c.depth.slow_down) {
      p.add(
          fmt(
              "casablanca.depth: hard_stop %.1f >= slow_down %.1f: no slow-down zone, the robot stops abruptly at the HIVE",
              c.depth.hard_stop, c.depth.slow_down));
    }
    if (c.side.hard_stop >= c.side.slow_down) {
      p.add(
          fmt(
              "casablanca.side: hard_stop %.1f >= slow_down %.1f: no slow-down zone, the robot stops abruptly at the HIVE",
              c.side.hard_stop, c.side.slow_down));
    }
    if (c.repulsion.decel_safety_factor <= 0) {
      p.add(
          "casablanca.repulsion.decel_safety_factor is 0: the robot will not drive toward the HIVE at all");
    }
    if (c.heading_lock.max_power > 1.0) {
      p.add(
          fmt(
              "casablanca.heading_lock.max_power %.2f > 1: limited to full power",
              c.heading_lock.max_power));
    }
  }

  private static void game(List<String> p) {
    var h = config.field.hive;
    if (config.hive.up_height_threshold_in <= h.down_opening_height_in
        || config.hive.up_height_threshold_in >= h.up_opening_height_in) {
      p.add(
          fmt(
              "hive.up_height_threshold_in %.1f must sit between the down (%.1f) and up (%.1f) CELL heights, or vision misreads every TIP",
              config.hive.up_height_threshold_in,
              h.down_opening_height_in,
              h.up_opening_height_in));
    }
    if (config.auto.park_reserve_ms >= config.match.auto_ms) {
      p.add(
          fmt(
              "auto.park_reserve_ms %d >= match.auto_ms %d: AUTO only parks",
              config.auto.park_reserve_ms, config.match.auto_ms));
    }
    var inv = config.inventory;
    if (inv.preload_count > inv.capacity_count) {
      p.add(
          fmt(
              "inventory.preload_count %d > capacity_count %d",
              inv.preload_count, inv.capacity_count));
    }
    if (config.auto.balls_per_volley_count > inv.capacity_count) {
      p.add(
          fmt(
              "auto.balls_per_volley_count %d > capacity_count %d: volleys wait for balls that cannot be held",
              config.auto.balls_per_volley_count, inv.capacity_count));
    }
  }

  private static String fmt(String format, Object... args) {
    return String.format(Locale.ROOT, format, args);
  }
}
