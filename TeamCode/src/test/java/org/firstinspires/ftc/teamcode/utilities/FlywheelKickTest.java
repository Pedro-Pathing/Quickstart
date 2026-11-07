package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayDeque;
import java.util.Random;
import net.jqwik.api.Assume;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.constraints.IntRange;
import net.jqwik.api.constraints.Scale;
import org.junit.jupiter.api.Test;

/** The recovery kick's model, predictor and power rule ({@link FlywheelKick}). */
class FlywheelKickTest {
  private static final double KS = 0.18;
  private static final double KV = 5e-4;
  private static final double NaN = Double.NaN;
  private static final double INF = Double.POSITIVE_INFINITY;

  /** Target 800 (stop at 0.97 = 776, trigger at 9% below = 728), tau 0.5 s, 20 ms loop. */
  private static double power(boolean kicking, double measured, double predicted, double normal) {
    return FlywheelKick.kickPower(
        kicking, measured, predicted, 800, 1.0, normal, 0.02, 0.5, KS, KV, 0.09, 0.97, 1.25);
  }

  @Test
  void steadySpeedIsZeroInsideFrictionAndLinearOutside() {
    assertEquals(0.0, FlywheelKick.steadySpeed(0.1, KS, KV));
    assertEquals(0.0, FlywheelKick.steadySpeed(-KS, KS, KV));
    assertEquals((0.68 - KS) / KV, FlywheelKick.steadySpeed(0.68, KS, KV), 1e-9);
    assertEquals(-(0.68 - KS) / KV, FlywheelKick.steadySpeed(-0.68, KS, KV), 1e-9);
    // A negative offset (the integrator found the wheel easier than calibrated) has no dead zone.
    assertEquals((0.05 + 0.02) / KV, FlywheelKick.steadySpeed(0.05, -0.02, KV), 1e-9);
    assertEquals(-(0.05 + 0.02) / KV, FlywheelKick.steadySpeed(-0.05, -0.02, KV), 1e-9);
  }

  @Test
  void steadySpeedRefusesBadInputs() {
    for (double kv : new double[] {0.0, -KV, NaN, INF}) {
      assertEquals(0.0, FlywheelKick.steadySpeed(0.5, KS, kv));
    }
    assertEquals(0.0, FlywheelKick.steadySpeed(NaN, KS, KV));
    assertEquals(0.0, FlywheelKick.steadySpeed(INF, KS, KV));
    assertEquals(0.0, FlywheelKick.steadySpeed(0.5, NaN, KV));
  }

  @Test
  void stepMovesTheGapFractionAndSaturatesAtTau() {
    double full = FlywheelKick.steadySpeed(1.0, KS, KV);
    assertEquals(100 + (full - 100) * 0.1, FlywheelKick.step(100, 1.0, 0.02, 0.2, KS, KV), 1e-9);
    assertEquals(full, FlywheelKick.step(100, 1.0, 0.5, 0.2, KS, KV), 1e-9);
    assertEquals(100, FlywheelKick.step(100, 1.0, 0.02, 0.0, KS, KV));
    assertEquals(100, FlywheelKick.step(100, 1.0, 0.0, 0.2, KS, KV));
    assertEquals(100, FlywheelKick.step(100, 1.0, NaN, 0.2, KS, KV));
  }

  @Test
  void farBelowTargetTheKickIsFullPower() {
    // A 1.25x-fast loop covers 5% of the gap: 640 needs a pull to 3360, far past full power.
    assertEquals(1.0, power(false, 640, 640, 0.6));
  }

  @Test
  void nearTargetItAsksOnlyForEnoughToReachTarget() {
    // From 760, a pull to 760 + 40 / 0.05 = 1560 ticks/s lands a 1.25x-fast wheel on 800.
    assertEquals(KS + KV * 1560, power(true, 760, 760, 0.6), 1e-12);
  }

  @Test
  void theKickEndsOnceTheNormalLoopAsksForAsMuch() {
    // From 770 the kick wants 0.18 + 5e-4 * (770 + 30 / 0.05) = 0.865.
    assertEquals(0.865, power(true, 770, 770, 0.80), 1e-12);
    assertTrue(Double.isNaN(power(true, 770, 770, 0.90)));
  }

  @Test
  void aKickStartsOnlyBelowTheTriggerButContinuesAboveIt() {
    assertTrue(Double.isNaN(power(false, 740, 740, 0.6)), "740 is above the 728 trigger");
    assertEquals(Math.min(1.0, KS + KV * (740 + 60 / 0.05)), power(true, 740, 740, 0.6), 1e-12);
  }

  @Test
  void nothingAtOrPastTheStop() {
    assertTrue(Double.isNaN(power(true, 600, 776, 0.6)));
    assertTrue(Double.isNaN(power(true, 600, 900, 0.6)));
  }

  @Test
  void theStopUsesThePredictedSpeedNotTheLaggedReading() {
    // The reading still says 640 (a ball), but by the model the wheel is already at 776.
    assertTrue(Double.isNaN(power(true, 640, 776, 0.6)));
  }

  @Test
  void speedMarginAsksForLessAndNeverForMoreThanTheExactModel() {
    // tau 0.25 s: one loop covers 8% of the gap (10% for a 1.25x-fast wheel).
    double exact =
        FlywheelKick.kickPower(true, 770, 770, 800, 1.0, 0.0, 0.02, 0.25, KS, KV, 0.09, 0.97, 1.0);
    double margin =
        FlywheelKick.kickPower(true, 770, 770, 800, 1.0, 0.0, 0.02, 0.25, KS, KV, 0.09, 0.97, 1.25);
    double under =
        FlywheelKick.kickPower(true, 770, 770, 800, 1.0, 0.0, 0.02, 0.25, KS, KV, 0.09, 0.97, 0.5);
    assertEquals(KS + KV * (770 + 30 / 0.08), exact, 1e-12);
    assertEquals(KS + KV * (770 + 30 / 0.10), margin, 1e-12);
    assertTrue(margin < exact);
    assertEquals(exact, under, 1e-12, "a margin under 1 is treated as 1");
  }

  @Test
  void neverKicksOnBadInputs() {
    double[][] bad = {
      {0, 0, 0, 1, 0.5, 0.02, 0.5, KS, KV}, // no target
      {0, 0, -800, 1, 0.5, 0.02, 0.5, KS, KV},
      {0, 0, 800, 1, 0.5, 0.02, 0.0, KS, KV}, // no tau
      {0, 0, 800, 1, 0.5, 0.0, 0.5, KS, KV}, // no dt
      {0, 0, 800, 1, 0.5, 0.02, 0.5, KS, 0.0}, // no kV
      {NaN, 0, 800, 1, 0.5, 0.02, 0.5, KS, KV},
      {0, NaN, 800, 1, 0.5, 0.02, 0.5, KS, KV},
      {0, 0, INF, 1, 0.5, 0.02, 0.5, KS, KV},
      {0, 0, 800, NaN, 0.5, 0.02, 0.5, KS, KV},
      {0, 0, 800, 1, NaN, 0.02, 0.5, KS, KV},
      {0, 0, 800, 1, 0.5, 0.02, 0.5, NaN, KV},
      {0, 0, 800, 1, 0.5, 0.02, 0.5, KS, INF},
    };
    for (double[] b : bad) {
      assertTrue(
          Double.isNaN(
              FlywheelKick.kickPower(
                  true, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], 0.09, 0.97, 1.25)));
    }
  }

  @Test
  void theLagMarginOnlyAppliesWhileKicking() {
    FlywheelKick kick = new FlywheelKick();
    kick.record(1.0, 0.02);
    kick.record(1.0, 0.02);
    // Not kicking: predicts over the 20 ms lag only, from 640, and starts a full-power kick.
    assertEquals(
        1.0, kick.update(640, 800, 1.0, 0.6, 0.02, 0.5, 0.02, 0.02, KS, KV, 0.09, 0.97, 1.0));
    double plain = kick.predictNow(640, 0.02, 0.5, KS, KV);
    double padded = kick.predictNow(640, 0.04, 0.5, KS, KV);
    assertTrue(padded > plain, "an accelerating wheel predicts faster over a longer lag");
    // Kicking: the 20 ms margin is added, so the power matches the padded prediction.
    double expected =
        FlywheelKick.kickPower(
            true, 640, padded, 800, 1.0, 0.6, 0.02, 0.5, KS, KV, 0.09, 0.97, 1.0);
    assertEquals(
        expected, kick.update(640, 800, 1.0, 0.6, 0.02, 0.5, 0.02, 0.02, KS, KV, 0.09, 0.97, 1.0));
  }

  @Test
  void noLagOrNoHistoryPredictsTheReading() {
    FlywheelKick kick = new FlywheelKick();
    assertEquals(500, kick.predictNow(500, 0.04, 0.5, KS, KV));
    kick.record(1.0, 0.02);
    assertEquals(500, kick.predictNow(500, 0.0, 0.5, KS, KV));
  }

  @Test
  void badRecordsAreIgnored() {
    FlywheelKick kick = new FlywheelKick();
    kick.record(NaN, 0.02);
    kick.record(1.0, 0.0);
    kick.record(1.0, -0.02);
    kick.record(1.0, INF);
    assertEquals(500, kick.predictNow(500, 0.04, 0.5, KS, KV));
  }

  @Test
  void aPartLoopAtTheLagWindowsStartIsReplayedForItsPartOnly() {
    FlywheelKick kick = new FlywheelKick();
    kick.record(1.0, 0.02);
    kick.record(0.5, 0.02);
    // Lag 0.03 s: the last 0.01 s of the full-power loop, then the whole 0.5 loop.
    double expected =
        FlywheelKick.step(FlywheelKick.step(500, 1.0, 0.01, 0.5, KS, KV), 0.5, 0.02, 0.5, KS, KV);
    assertEquals(expected, kick.predictNow(500, 0.03, 0.5, KS, KV), 1e-9);
  }

  @Test
  void updateRemembersTheKickAndResetEndsIt() {
    FlywheelKick kick = new FlywheelKick();
    kick.record(1.0, 0.02);
    double p = kick.update(400, 800, 1.0, 0.6, 0.02, 0.5, 0.02, 0.005, KS, KV, 0.09, 0.97, 1.25);
    assertEquals(1.0, p);
    assertTrue(kick.isKicking());
    kick.reset();
    assertFalse(kick.isKicking());
    assertEquals(400, kick.predictNow(400, 0.04, 0.5, KS, KV));
  }

  /**
   * Oracle: a first-order wheel driven by random powers, read through an exact delay of {@code
   * lagLoops} loops. Given the true model, the predictor recovers the wheel's speed now from the
   * delayed reading, for any lag the history can hold.
   */
  @Property(tries = 300)
  void thePredictorUndoesAnExactDelay(
      @ForAll @IntRange(min = 0, max = 20) int lagLoops,
      @ForAll @DoubleRange(min = 0.1, max = 1.5) double tau,
      @ForAll @DoubleRange(min = 0.005, max = 0.03) @Scale(4) double dt,
      @ForAll long seed) {
    Random random = new Random(seed);
    FlywheelKick kick = new FlywheelKick();
    ArrayDeque<Double> speeds = new ArrayDeque<>();
    double speed = 300 * random.nextDouble();
    speeds.addLast(speed);
    for (int i = 0; i < 40; i++) {
      double u = random.nextDouble();
      speed = FlywheelKick.step(speed, u, dt, tau, KS, KV);
      kick.record(u, dt);
      speeds.addLast(speed);
      while (speeds.size() > lagLoops + 1) {
        speeds.removeFirst();
      }
    }
    double delayed = speeds.peekFirst();
    assertEquals(speed, kick.predictNow(delayed, lagLoops * dt, tau, KS, KV), 1e-6 * (1 + speed));
  }

  /**
   * Oracle: on a wheel that is exactly the model (margin 1), a partial kick lands one loop later
   * exactly on target, and full power never passes it.
   */
  @Property(tries = 500)
  void aPartialKickLandsExactlyOnTarget(
      @ForAll @DoubleRange(min = 100, max = 770) double predicted,
      @ForAll @DoubleRange(min = 0.1, max = 1.5) double tau,
      @ForAll @DoubleRange(min = 0.005, max = 0.03) @Scale(4) double dt,
      @ForAll @DoubleRange(min = -0.1, max = 0.3) double ks) {
    double p =
        FlywheelKick.kickPower(
            true, predicted, predicted, 800, 1.0, -1.0, dt, tau, ks, KV, 0.09, 0.97, 1.0);
    Assume.that(!Double.isNaN(p));
    double next = FlywheelKick.step(predicted, p, dt, tau, ks, KV);
    if (p < 1.0) {
      assertEquals(800.0, next, 1e-6);
    } else {
      assertTrue(next <= 800.0 + 1e-6, "full power passed target: " + next);
    }
  }
}
