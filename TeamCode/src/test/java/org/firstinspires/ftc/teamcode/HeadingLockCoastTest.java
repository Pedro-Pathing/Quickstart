package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.junit.jupiter.api.Test;

/**
 * The heading lock stops pushing once the measured braking model says the chassis will coast onto
 * its target. Without that, the friction step saturates the output at the deadband edge and the
 * lock overshoots: at 1.2 rad/s the chassis coasts ~4.0 deg, the whole width of a +/-2 deg
 * deadband, and at 1.5 rad/s ~5 deg, clean across it.
 */
public class HeadingLockCoastTest {

  private static double coastRad(double omega) {
    return Math.abs(
        Casablanca.brakingDistance(
            omega, config.pedro.braking.heading.linear, config.pedro.braking.heading.quadratic));
  }

  @Test
  public void measuredBrakingModelCoversTheDeadbandAtModerateTurnRates() {
    double deadbandRad = Math.toRadians(config.casablanca.heading_lock.error_deadband_deg);
    assertTrue(
        coastRad(1.2) >= 0.95 * 2 * deadbandRad,
        "documented failure mode: coast at 1.2 rad/s is about the full deadband width");
    assertTrue(coastRad(1.5) > 2 * deadbandRad, "and at 1.5 rad/s it overshoots clean across it");
  }

  @Test
  public void coastsWhenClosingFastEnough() {
    double error = Math.toRadians(3.0);
    double omega = 1.2;
    assertTrue(coastRad(omega) >= error);
    assertTrue(Casablanca.willCoastToHeading(error, omega));
    assertTrue(Casablanca.willCoastToHeading(-error, -omega));
  }

  @Test
  public void keepsPushingWhenSlowOrTurningAway() {
    double error = Math.toRadians(10.0);
    assertFalse(Casablanca.willCoastToHeading(error, 0.5), "too slow to coast 10 deg");
    assertFalse(Casablanca.willCoastToHeading(error, -2.0), "turning away from the target");
    assertFalse(Casablanca.willCoastToHeading(error, 0.0), "stationary");
    assertFalse(Casablanca.willCoastToHeading(0.0, 1.0));
  }

  @Test
  public void coastThresholdMatchesTheBrakingModelExactly() {
    double omega = 0.8;
    double coast = coastRad(omega);
    assertTrue(Casablanca.willCoastToHeading(coast, omega));
    assertFalse(Casablanca.willCoastToHeading(coast + 1e-6, omega));
    assertEquals(coast, coastRad(-omega), 1e-12);
  }
}
