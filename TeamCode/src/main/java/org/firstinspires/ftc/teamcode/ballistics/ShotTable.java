package org.firstinspires.ftc.teamcode.ballistics;

import java.util.Locale;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

public final class ShotTable {

  /** One interpolated shot: what to command, and whether the distance was actually calibrated. */
  public record Shot(
      double hoodServoPosition, double rpm, boolean withinCalibratedRange, String reason) {}

  private final double[] distances;
  private final double[] hoodServo;
  private final double[] rpm;
  private final boolean calibrated;

  private ShotTable(double[] distances, double[] hoodServo, double[] rpm, boolean calibrated) {
    this.distances = distances;
    this.hoodServo = hoodServo;
    this.rpm = rpm;
    this.calibrated = calibrated;
  }

  /**
   * Builds the table for one SCORING ELEMENT from the flat {@code distance, hood_servo, rpm}
   * triples in config. POLLEN (2.8 in) and NECTAR (3.6 in, heavier) leave the shooter differently,
   * so each has its own rows.
   *
   * @throws IllegalStateException if the row data is malformed, since a silently mis-parsed shot
   *     table would command arbitrary hood and flywheel values on every shot of a match.
   */
  public static ShotTable fromConfig(BallType ballType) {
    var table = config.shooter.shot_table;
    double[] points = ballType == BallType.NECTAR ? table.nectar_points : table.pollen_points;
    return fromPoints(points, table.calibrated);
  }

  /** Builds a calibrated table from flat {@code distance, hood_servo, rpm} triples. */
  public static ShotTable fromPoints(double[] points) {
    return fromPoints(points, true);
  }

  /**
   * Builds a table from flat {@code distance, hood_servo, rpm} triples.
   *
   * @param calibrated false for placeholder rows, which the solver never treats as a valid shot
   */
  public static ShotTable fromPoints(double[] points, boolean calibrated) {
    if (points == null || points.length == 0) {
      throw new IllegalStateException(
          "shooter.shot_table is empty — run the Ballistics Calibration OpMode");
    }
    if (points.length % 3 != 0) {
      throw new IllegalStateException(
          "shooter.shot_table rows must be flat distance,hood_servo,rpm triples but had "
              + points.length
              + " values");
    }

    int n = points.length / 3;
    double[] d = new double[n];
    double[] h = new double[n];
    double[] r = new double[n];
    for (int i = 0; i < n; i++) {
      d[i] = points[i * 3];
      h[i] = points[i * 3 + 1];
      r[i] = points[i * 3 + 2];
      if (!Double.isFinite(d[i]) || !Double.isFinite(h[i]) || !Double.isFinite(r[i])) {
        throw new IllegalStateException("shot table row " + i + " has a non-finite value");
      }
      if (i > 0 && d[i] <= d[i - 1]) {
        throw new IllegalStateException(
            String.format(
                Locale.ROOT,
                "shot table distances must strictly increase, but row %d (%.1f in) does not follow"
                    + " row %d (%.1f in)",
                i,
                d[i],
                i - 1,
                d[i - 1]));
      }
    }
    return new ShotTable(d, h, r, calibrated);
  }

  /** False while the rows are placeholders that must not drive an automatic feed. */
  public boolean isCalibrated() {
    return calibrated;
  }

  /** Lowest calibrated distance (inches). */
  public double minDistance() {
    return distances[0];
  }

  /** Highest calibrated distance (inches). */
  public double maxDistance() {
    return distances[distances.length - 1];
  }

  public int size() {
    return distances.length;
  }

  /**
   * Looks up the hood position and flywheel setpoint for a distance, interpolating between the two
   * bracketing rows. Outside the calibrated range the nearest end row is returned with {@code
   * withinCalibratedRange == false} so callers can gate on it rather than acting on a guess.
   */
  public Shot lookup(double distanceInches) {
    if (!Double.isFinite(distanceInches)) {
      return new Shot(hoodServo[0], rpm[0], false, "Non-finite shot distance");
    }
    if (distanceInches <= distances[0]) {
      return new Shot(
          hoodServo[0],
          rpm[0],
          distanceInches >= distances[0] - 1e-9,
          String.format(
              Locale.ROOT,
              "%.1f in is below the calibrated range (min %.1f in)",
              distanceInches,
              distances[0]));
    }
    int last = distances.length - 1;
    if (distanceInches >= distances[last]) {
      return new Shot(
          hoodServo[last],
          rpm[last],
          distanceInches <= distances[last] + 1e-9,
          String.format(
              Locale.ROOT,
              "%.1f in is beyond the calibrated range (max %.1f in)",
              distanceInches,
              distances[last]));
    }

    int hi = 1;
    while (hi < last && distances[hi] < distanceInches) {
      hi++;
    }
    int lo = hi - 1;
    double t = (distanceInches - distances[lo]) / (distances[hi] - distances[lo]);
    return new Shot(
        hoodServo[lo] + t * (hoodServo[hi] - hoodServo[lo]),
        rpm[lo] + t * (rpm[hi] - rpm[lo]),
        true,
        "Valid");
  }
}
