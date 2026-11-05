package org.firstinspires.ftc.teamcode.ballistics;

import androidx.annotation.Nullable;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Autonomous shoot windows keyed by shot distance: how long one volley may take before the routine
 * gives up on it and moves on.
 *
 * <p>Rows are flat {@code distance_in, window_ms} pairs in {@code auto.shot_time_points}, in
 * exactly the format the Shot Timing Tuner OpMode exports. (The DECODE version read RPM-keyed rows
 * from a different key than the tuner wrote, so tuner output could never be used without
 * hand-editing.)
 */
public final class ShotTimeTable {

  private final double[] distances;
  private final int[] windowMs;

  private ShotTimeTable(double[] distances, int[] windowMs) {
    this.distances = distances;
    this.windowMs = windowMs;
  }

  /**
   * Builds the table from config, or returns null when none is configured so the caller can fall
   * back to {@code auto.shoot_wait_ms}.
   *
   * @throws IllegalStateException if the rows are malformed, since a mis-parsed table would bound
   *     every autonomous volley with an arbitrary number.
   */
  public static @Nullable ShotTimeTable fromConfig() {
    return fromPoints(config.auto.shot_time_points);
  }

  /** Builds a table from flat {@code distance_in, window_ms} pairs; null when empty. */
  public static @Nullable ShotTimeTable fromPoints(@Nullable double[] points) {
    if (points == null || points.length == 0) {
      return null;
    }
    if (points.length % 2 != 0) {
      throw new IllegalStateException(
          "auto.shot_time_points must be flat distance_in,window_ms pairs but had "
              + points.length
              + " values");
    }

    int n = points.length / 2;
    double[] d = new double[n];
    int[] w = new int[n];
    for (int i = 0; i < n; i++) {
      d[i] = points[i * 2];
      w[i] = (int) Math.round(points[i * 2 + 1]);
      if (!Double.isFinite(d[i])) {
        throw new IllegalStateException("shot time table distance in row " + i + " is not finite");
      }
      if (i > 0 && d[i] <= d[i - 1]) {
        throw new IllegalStateException(
            String.format(
                Locale.ROOT,
                "shot time table distances must strictly increase, but row %d (%.1f in) does not"
                    + " follow row %d (%.1f in)",
                i,
                d[i],
                i - 1,
                d[i - 1]));
      }
      if (w[i] < 0) {
        throw new IllegalStateException(
            String.format(
                Locale.ROOT, "shot time table window at %.1f in is negative (%d ms)", d[i], w[i]));
      }
    }
    return new ShotTimeTable(d, w);
  }

  public int size() {
    return distances.length;
  }

  public double minDistance() {
    return distances[0];
  }

  public double maxDistance() {
    return distances[distances.length - 1];
  }

  /** Shoot window (ms) for a distance, interpolated between rows and clamped outside them. */
  public int lookupMs(double distanceInches) {
    if (!Double.isFinite(distanceInches) || distanceInches <= distances[0]) {
      return windowMs[0];
    }
    int last = distances.length - 1;
    if (distanceInches >= distances[last]) {
      return windowMs[last];
    }

    int hi = 1;
    while (hi < last && distances[hi] < distanceInches) {
      hi++;
    }
    int lo = hi - 1;
    double t = (distanceInches - distances[lo]) / (distances[hi] - distances[lo]);
    return (int) Math.round(windowMs[lo] + t * (windowMs[hi] - windowMs[lo]));
  }

  /**
   * Window for a shot distance, falling back to {@code auto.shoot_wait_ms} when no table is
   * configured or it is malformed. Never throws and never returns zero, because a volley bounded by
   * nothing is an autonomous routine that never ends.
   */
  public static int windowMsFor(double distanceInches) {
    int fallback = Math.max(1, config.auto.shoot_wait_ms);
    try {
      ShotTimeTable table = fromConfig();
      int window = table != null ? table.lookupMs(distanceInches) : fallback;
      return window > 0 ? window : fallback;
    } catch (RuntimeException e) {
      return fallback;
    }
  }
}
