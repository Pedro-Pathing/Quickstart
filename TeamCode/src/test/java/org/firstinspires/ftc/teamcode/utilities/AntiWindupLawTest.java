package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The pure functions of {@link AntiWindupIntegrator}, which OpenJML proves total (no throw). */
class AntiWindupLawTest {
  private static final double NaN = Double.NaN;
  private static final double INF = Double.POSITIVE_INFINITY;

  @Test
  void nanAndNonPositiveGainClearAndContributeNothing() {
    for (double ki : new double[] {NaN, 0.0, -1.0, -INF}) {
      assertEquals(0.0, AntiWindupIntegrator.nextAccumulator(5.0, 10, 0.02, ki, 100, 0.2, 0));
      assertEquals(0.0, AntiWindupIntegrator.contribution(5.0, ki, 0.2));
    }
  }

  @Test
  void infiniteGainAndInfiniteLimitNoLongerThrow() {
    // Math.clamp(x, -NaN, NaN) threw here: inf / inf is NaN.
    double next = AntiWindupIntegrator.nextAccumulator(1.0, 1.0, 0.02, INF, 0, INF, 0);
    assertFalse(Double.isNaN(next));
    assertEquals(0.0, AntiWindupIntegrator.contribution(0.0, INF, 0.2));
  }

  @Test
  void nonFiniteErrorDoesNotPoisonTheAccumulator() {
    for (double e : new double[] {NaN, INF, -INF}) {
      assertEquals(0.5, AntiWindupIntegrator.nextAccumulator(0.5, e, 0.02, 0.001, 0, 0.2, 0));
    }
  }

  @Test
  void nanAccumulatorIsTreatedAsZero() {
    assertEquals(0.2, AntiWindupIntegrator.nextAccumulator(NaN, 10, 0.02, 0.001, 0, 0.2, 0), 1e-12);
  }

  @Test
  void dtOutsideTheUsableWindowDoesNotIntegrate() {
    for (double dt : new double[] {0.0, -0.01, 0.2500001, 1e9, INF, NaN}) {
      assertFalse(AntiWindupIntegrator.mayIntegrate(10, dt, 0, 0), "dt=" + dt);
      assertEquals(0.3, AntiWindupIntegrator.nextAccumulator(0.3, 10, dt, 0.001, 0, 0.2, 0), 1e-12);
    }
    assertTrue(AntiWindupIntegrator.mayIntegrate(10, AntiWindupIntegrator.MAX_DT_SECONDS, 0, 0));
  }

  @Test
  void errorOutsideTheBandDoesNotIntegrate() {
    assertFalse(AntiWindupIntegrator.mayIntegrate(151, 0.02, 150, 0));
    assertTrue(AntiWindupIntegrator.mayIntegrate(150, 0.02, 150, 0));
    assertTrue(AntiWindupIntegrator.mayIntegrate(1e9, 0.02, 0, 0), "band <= 0 means always");
  }

  @Test
  void neverChargesFurtherIntoARail() {
    assertFalse(AntiWindupIntegrator.mayIntegrate(5, 0.02, 0, 1));
    assertFalse(AntiWindupIntegrator.mayIntegrate(-5, 0.02, 0, -1));
    assertTrue(AntiWindupIntegrator.mayIntegrate(-5, 0.02, 0, 1), "unwinding is allowed");
    assertEquals(0.3, AntiWindupIntegrator.nextAccumulator(0.3, 5, 0.02, 0.001, 0, 0.2, 1), 1e-12);
  }

  @Test
  void contributionStaysWithinMaxEvenForHugeAccumulators() {
    for (double acc : new double[] {1e300, -1e300, INF, -INF, 0.1, -0.1}) {
      double c = AntiWindupIntegrator.contribution(acc, 0.0005, 0.15);
      assertTrue(Math.abs(c) <= 0.15, "acc=" + acc + " c=" + c);
    }
    double next = AntiWindupIntegrator.nextAccumulator(1e300, 10, 0.02, 0.0005, 0, 0.15, 0);
    assertTrue(Math.abs(AntiWindupIntegrator.contribution(next, 0.0005, 0.15)) <= 0.15);
  }

  @Test
  void nonPositiveMaxContributionMeansUnbounded() {
    assertEquals(Double.MAX_VALUE, AntiWindupIntegrator.accumulatorLimit(0.001, 0.0));
    assertEquals(Double.MAX_VALUE, AntiWindupIntegrator.accumulatorLimit(0.001, NaN));
    assertEquals(150.0, AntiWindupIntegrator.accumulatorLimit(0.001, 0.15), 1e-9);
  }
}
