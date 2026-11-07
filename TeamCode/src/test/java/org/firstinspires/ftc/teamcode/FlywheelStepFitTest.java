package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.constraints.Scale;
import org.firstinspires.ftc.teamcode.ballistics.FlywheelStepFit;
import org.junit.jupiter.api.Test;

/** {@link FlywheelStepFit}: the least-squares first-order-plus-dead-time fit. */
class FlywheelStepFitTest {

  /** A first-order wheel with dead time, sampled every {@code dt}, with uniform noise. */
  private static double[][] step(
      double v0,
      double vFinal,
      double tau,
      double lag,
      double dt,
      double durationSec,
      double noise,
      long seed) {
    Random random = new Random(seed);
    int n = (int) (durationSec / dt);
    double[] t = new double[n];
    double[] v = new double[n];
    for (int i = 0; i < n; i++) {
      t[i] = (i + 1) * dt;
      double moved = t[i] <= lag ? 0.0 : 1.0 - Math.exp(-(t[i] - lag) / tau);
      v[i] = v0 + (vFinal - v0) * moved + noise * (2 * random.nextDouble() - 1);
    }
    return new double[][] {t, v};
  }

  @Test
  void recoversTauAndLagFromAFineCleanStep() {
    double[][] s = step(400, 1100, 0.5, 0.04, 0.001, 4.0, 0.0, 1);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 400, 1100);
    assertTrue(fit.valid());
    assertEquals(0.5, fit.tauSec(), 0.005);
    assertEquals(0.04, fit.lagSec(), 0.003);
  }

  @Test
  void aFallingStepFitsTheSame() {
    double[][] s = step(1100, 400, 0.3, 0.02, 0.001, 3.0, 0.0, 1);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 1100, 400);
    assertTrue(fit.valid());
    assertEquals(0.3, fit.tauSec(), 0.005);
    assertEquals(0.02, fit.lagSec(), 0.003);
  }

  @Test
  void refusesTooSmallAStep() {
    double[][] s = step(800, 820, 0.5, 0.0, 0.01, 3.0, 0.0, 1);
    assertFalse(FlywheelStepFit.fit(s[0], s[1], 800, 820).valid());
    assertFalse(FlywheelStepFit.fit(s[0], s[1], 800, Double.NaN).valid());
  }

  @Test
  void refusesAStepCutShort() {
    // 0.25 s of a 1 s wheel reaches 22% of the step: under five samples in the window.
    double[][] s = step(400, 1100, 1.0, 0.0, 0.01, 0.25, 0.0, 1);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 400, 1100);
    assertFalse(fit.valid());
    assertTrue(Double.isNaN(fit.tauSec()));
  }

  @Test
  void refusesMismatchedArrays() {
    assertFalse(FlywheelStepFit.fit(new double[3], new double[2], 0, 500).valid());
  }

  @Test
  void aSingleSpikeBarelyMovesTheFit() {
    double[][] s = step(400, 1100, 0.5, 0.0, 0.01, 4.0, 0.0, 1);
    FlywheelStepFit.Result clean = FlywheelStepFit.fit(s[0], s[1], 400, 1100);
    s[1][2] = 1100; // a full-step spike at 30 ms: not a confirmed crossing, one sample of ~150
    FlywheelStepFit.Result spiked = FlywheelStepFit.fit(s[0], s[1], 400, 1100);
    assertEquals(clean.tauSec(), spiked.tauSec(), 0.02 * clean.tauSec());
    assertEquals(clean.lagSec(), spiked.lagSec(), 0.005);
  }

  @Test
  void aZeroLagNeverReportsNegative() {
    double[][] s = step(400, 1100, 0.5, 0.0, 0.02, 4.0, 0.0, 1);
    assertTrue(FlywheelStepFit.fit(s[0], s[1], 400, 1100).lagSec() >= 0.0);
  }

  /**
   * At the characterization step test's loop (a few ms: it only reads the hub) with realistic
   * encoder noise (up to 15 ticks/s on a step of 500+), tau comes back within 3% and the lag within
   * 7.5 ms, inside the error FlywheelClosedLoopProperties gives the kick (and kick_lag_margin_sec).
   */
  @Property(tries = 300)
  void fitsAnyPlausibleWheelAtTheOpModesLoopRate(
      @ForAll @DoubleRange(min = 0.15, max = 1.0) double tau,
      @ForAll @DoubleRange(min = 0.0, max = 0.06) double lag,
      @ForAll @DoubleRange(min = 0.002, max = 0.010) @Scale(4) double dt,
      @ForAll @DoubleRange(min = 500, max = 900) double rise,
      @ForAll @DoubleRange(min = 0, max = 15) double noise,
      @ForAll long seed) {
    double[][] s = step(400, 400 + rise, tau, lag, dt, 5 * tau + 0.5, noise, seed);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 400, 400 + rise);
    assertTrue(fit.valid(), fit.problem());
    assertEquals(tau, fit.tauSec(), 0.03 * tau, "tau");
    assertEquals(lag, fit.lagSec(), 0.0075, "lag");
  }
}
