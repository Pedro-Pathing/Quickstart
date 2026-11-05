package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * How fast the turret gets there, over a fixed grid of plants (gain, friction around the
 * calibration, servo lag) and moves, plus the CPU cost of one {@link Turret#periodic()}. Not a pass
 * or fail test: run with {@code ./gradlew :TeamCode:testDebugUnitTest -Pbenchmark --tests
 * '*TurretTimingBenchmark*'} and compare {@code build/reports/turret-timing.txt} across changes.
 */
@Tag("benchmark")
class TurretTimingBenchmark {
  private static final double[] GAINS = {80, 140, 200};
  private static final double[] FRICTION_SCALES = {0.7, 1.0, 1.3};
  private static final double[] LAGS = {0.0, 0.02, 0.04};
  private static final double[][] MOVES = {
    {-24, 24}, {24, -24}, {0, 12}, {12, 0}, {-6, -2}, {2, 6}, {-20, -16}, {16, 20}
  };
  private static final double DISTANCE_IN = 60;
  private static final double RUN_SEC = 4.0;

  @Test
  void report() throws IOException {
    boolean savedEnabled = config.turret.enabled;
    config.turret.enabled = true;
    try {
      List<Double> firstAimed = new ArrayList<>();
      List<Double> settled = new ArrayList<>();
      List<Double> overshoot = new ArrayList<>();
      int neverSettled = 0;
      long periodicNanos = 0;
      long periodicCalls = 0;
      long seed = 1;
      for (double k : GAINS) {
        for (double scale : FRICTION_SCALES) {
          for (double lag : LAGS) {
            for (double[] move : MOVES) {
              TurretSim.Plant plant =
                  new TurretSim.Plant(
                      k,
                      config.turret.ks_positive * scale,
                      config.turret.ks_negative * scale,
                      lag,
                      0.05,
                      0.003,
                      seed++);
              TurretSim sim = new TurretSim(plant, move[0], new Pose(40, 40, 0));
              sim.aimAtBearing(DISTANCE_IN, move[1]);
              double direction = Math.signum(move[1] - move[0]);
              double first = Double.NaN;
              double lastUnaimed = 0;
              double over = 0;
              int loops = (int) (RUN_SEC / TurretSim.LOOP_SEC);
              for (int i = 1; i <= loops; i++) {
                long t0 = System.nanoTime();
                sim.step();
                periodicNanos += System.nanoTime() - t0;
                periodicCalls++;
                double t = i * TurretSim.LOOP_SEC;
                boolean aimed = sim.turret.isAimed(sim.pose);
                if (aimed && Double.isNaN(first)) {
                  first = t;
                }
                if (!aimed) {
                  lastUnaimed = t;
                }
                over = Math.max(over, direction * (sim.angle - move[1]));
              }
              if (Double.isNaN(first) || lastUnaimed >= RUN_SEC - TurretSim.LOOP_SEC) {
                neverSettled++;
              } else {
                firstAimed.add(first);
                settled.add(lastUnaimed + TurretSim.LOOP_SEC);
              }
              overshoot.add(over);
            }
          }
        }
      }
      int runs = GAINS.length * FRICTION_SCALES.length * LAGS.length * MOVES.length;
      String report =
          String.format(
              Locale.ROOT,
              "turret timing over %d runs (%d never settled in %.0f s)%n"
                  + "  first aimed (s):    %s%n"
                  + "  settled for good (s): %s%n"
                  + "  overshoot (deg):    %s%n"
                  + "  sim step incl. periodic(): %.1f us/loop%n",
              runs,
              neverSettled,
              RUN_SEC,
              summary(firstAimed),
              summary(settled),
              summary(overshoot),
              periodicNanos / 1e3 / periodicCalls);
      System.out.print(report);
      Path out = Path.of("build", "reports", "turret-timing.txt");
      Files.createDirectories(out.getParent());
      Files.writeString(out, report);
    } finally {
      config.turret.enabled = savedEnabled;
    }
  }

  /**
   * Every corner of the plant box TurretClosedLoopProperties samples from (gain, friction either
   * way, lag, noise, loop jitter) at short and long range: lists any run that is not settled for
   * good, i.e. still hunting or unaimed over the last two seconds of six.
   */
  @Test
  void cornerSweep() throws IOException {
    boolean savedEnabled = config.turret.enabled;
    config.turret.enabled = true;
    try {
      double[] frictions = {0.0, 0.08, config.turret.ks_negative * 1.3 + 0.01};
      StringBuilder bad = new StringBuilder();
      int runs = 0;
      int failures = 0;
      double worstSettle = 0;
      long seed = 7;
      for (double k : new double[] {80, 200}) {
        for (double sPos : frictions) {
          for (double sNeg : frictions) {
            for (double lag : new double[] {0.0, 0.04}) {
              for (double noise : new double[] {0.0, 0.15}) {
                for (double distance : new double[] {20, 140, 190}) {
                  for (double[] move : MOVES) {
                    runs++;
                    TurretSim.Plant plant =
                        new TurretSim.Plant(k, sPos, sNeg, lag, noise, 0.006, seed++);
                    TurretSim sim = new TurretSim(plant, move[0], new Pose(40, 40, 0));
                    sim.aimAtBearing(distance, move[1]);
                    int loops = (int) (6.0 / TurretSim.LOOP_SEC);
                    double lastUnaimed = 0;
                    int reversalsLate = 0;
                    double lastSign = 0;
                    boolean faulted = false;
                    for (int i = 1; i <= loops; i++) {
                      sim.step();
                      faulted |= sim.turret.isFaulted();
                      double t = i * TurretSim.LOOP_SEC;
                      if (!sim.turret.isAimed(sim.pose)) {
                        lastUnaimed = t;
                      }
                      double sign = Math.signum(sim.power);
                      if (t > 4.0 && sign != 0 && lastSign != 0 && sign != lastSign) {
                        reversalsLate++;
                      }
                      if (sign != 0) {
                        lastSign = sign;
                      }
                    }
                    boolean ok = lastUnaimed <= 4.0 && reversalsLate <= 2 && !faulted;
                    if (ok) {
                      worstSettle = Math.max(worstSettle, lastUnaimed);
                    } else {
                      failures++;
                      bad.append(
                          String.format(
                              Locale.ROOT,
                              "  k=%.0f s+=%.3f s-=%.3f lag=%.2f noise=%.2f d=%.0f %+.0f->%+.0f:"
                                  + " last unaimed %.2f s, late reversals %d, faulted %s%n",
                              k,
                              sPos,
                              sNeg,
                              lag,
                              noise,
                              distance,
                              move[0],
                              move[1],
                              lastUnaimed,
                              reversalsLate,
                              faulted));
                    }
                  }
                }
              }
            }
          }
        }
      }
      String report =
          String.format(
              Locale.ROOT,
              "corner sweep: %d runs, %d not settled for good; worst settle %.2f s%n%s",
              runs,
              failures,
              worstSettle,
              bad);
      System.out.print(report);
      Path out = Path.of("build", "reports", "turret-corners.txt");
      Files.createDirectories(out.getParent());
      Files.writeString(out, report);
    } finally {
      config.turret.enabled = savedEnabled;
    }
  }

  private static String summary(List<Double> values) {
    if (values.isEmpty()) {
      return "none";
    }
    double[] v = values.stream().mapToDouble(Double::doubleValue).sorted().toArray();
    return String.format(
        Locale.ROOT,
        "median %.3f  p95 %.3f  max %.3f  mean %.3f",
        v[v.length / 2],
        v[(int) Math.min(v.length - 1, Math.ceil(v.length * 0.95) - 1)],
        v[v.length - 1],
        Arrays.stream(v).average().orElse(Double.NaN));
  }
}
