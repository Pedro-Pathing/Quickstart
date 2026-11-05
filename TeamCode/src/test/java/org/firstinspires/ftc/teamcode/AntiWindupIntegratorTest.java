package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.utilities.AntiWindupIntegrator;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AntiWindupIntegratorTest {

  private static final double KI = 0.0005;
  private static final double BAND = 150.0;
  private static final double MAX_CONTRIBUTION = 0.15;
  private static final double DT = 0.02;

  private AntiWindupIntegrator integrator;

  @BeforeEach
  public void setUp() {
    integrator = new AntiWindupIntegrator();
  }

  private double step(double error) {
    return integrator.update(error, DT, KI, BAND, MAX_CONTRIBUTION, 0);
  }

  private double step(double error, int saturationSign) {
    return integrator.update(error, DT, KI, BAND, MAX_CONTRIBUTION, saturationSign);
  }

  @Test
  public void testAccumulatesInsideBand() {
    double contribution = step(100.0);
    assertEquals(KI * 100.0 * DT, contribution, 1e-12);
    assertEquals(100.0 * DT, integrator.getAccumulator(), 1e-12);

    step(100.0);
    assertEquals(2 * 100.0 * DT, integrator.getAccumulator(), 1e-12);
  }

  /** The spin-up case: error far outside the band must not charge the integrator at all. */
  @Test
  public void testDoesNotChargeDuringSpinUp() {
    for (int i = 0; i < 100; i++) {
      step(870.0);
    }
    assertEquals(0.0, integrator.getAccumulator(), 1e-12);
    assertEquals(0.0, step(870.0), 1e-12);
  }

  /** Outside the band the accumulator holds, since it still encodes real feedforward error. */
  @Test
  public void testHoldsRatherThanZeroesOutsideBand() {
    step(100.0);
    step(100.0);
    double held = integrator.getAccumulator();
    assertTrue(held > 0.0);

    step(900.0); // large transient, e.g. a ball strike
    assertEquals(held, integrator.getAccumulator(), 1e-12);
  }

  @Test
  public void testContributionIsClampedToMaxContribution() {
    for (int i = 0; i < 5000; i++) {
      step(150.0);
    }
    assertEquals(MAX_CONTRIBUTION, step(150.0), 1e-9);
    assertTrue(
        integrator.getAccumulator() <= MAX_CONTRIBUTION / KI, "accumulator itself must be bounded");
  }

  @Test
  public void testClampIsSymmetricForNegativeError() {
    for (int i = 0; i < 5000; i++) {
      step(-150.0);
    }
    assertEquals(-MAX_CONTRIBUTION, step(-150.0), 1e-9);
  }

  /** Charging further into a rail cannot move the output, so that direction must freeze. */
  @Test
  public void testFreezesWhenPushingIntoSaturation() {
    step(100.0);
    double before = integrator.getAccumulator();

    step(100.0, 1); // positive error while already railed high
    assertEquals(before, integrator.getAccumulator(), 1e-12);
  }

  @Test
  public void testStillUnwindsAwayFromSaturation() {
    step(100.0);
    step(100.0);
    double before = integrator.getAccumulator();

    // Negative error while railed high is the recovery direction — must still integrate.
    step(-100.0, 1);
    assertTrue(integrator.getAccumulator() < before);
  }

  @Test
  public void testZeroGainDisablesAndClearsTerm() {
    step(100.0);
    assertTrue(integrator.getAccumulator() > 0.0);

    assertEquals(0.0, integrator.update(100.0, DT, 0.0, BAND, MAX_CONTRIBUTION, 0), 1e-12);
    assertEquals(0.0, integrator.getAccumulator(), 1e-12);
  }

  @Test
  public void testRejectsImplausibleDt() {
    // A scheduling stall must not dump a huge slab of error-seconds into the accumulator.
    integrator.update(100.0, 5.0, KI, BAND, MAX_CONTRIBUTION, 0);
    assertEquals(0.0, integrator.getAccumulator(), 1e-12);

    integrator.update(100.0, 0.0, KI, BAND, MAX_CONTRIBUTION, 0);
    assertEquals(0.0, integrator.getAccumulator(), 1e-12);

    integrator.update(100.0, -0.02, KI, BAND, MAX_CONTRIBUTION, 0);
    assertEquals(0.0, integrator.getAccumulator(), 1e-12);
  }

  @Test
  public void testResetClearsAccumulator() {
    step(100.0);
    step(100.0);
    assertTrue(integrator.getAccumulator() > 0.0);

    integrator.reset();
    assertEquals(0.0, integrator.getAccumulator(), 1e-12);
  }

  @Test
  public void testNonPositiveBandMeansAlwaysIntegrate() {
    double contribution = integrator.update(870.0, DT, KI, 0.0, MAX_CONTRIBUTION, 0);
    assertEquals(KI * 870.0 * DT, contribution, 1e-12);
  }

  /**
   * The regression this class exists to prevent: Pedro's unbounded integrator charges through the
   * whole spin-up, and the stored value then commands a large overshoot the flywheel cannot brake
   * off. The bounded version stays within its configured authority.
   */
  @Test
  public void testBoundedIntegratorCannotDominateCommandDuringSpinUp() {
    double pedroStyleAccumulator = 0.0;
    for (int i = 0; i < 50; i++) { // ~1 second of spin-up at 870 ticks/s error
      pedroStyleAccumulator += 870.0 * DT;
      step(870.0);
    }

    double pedroContribution = KI * pedroStyleAccumulator;
    assertTrue(pedroContribution > 0.4, "unbounded integral would saturate the motor on its own");
    assertEquals(0.0, step(870.0), 1e-12, "bounded integral contributes nothing during spin-up");
  }
}
