package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.junit.jupiter.api.Test;

/**
 * Covers {@link Shooter#voltageCompensationScale}. The flywheel's kS/kV feedforward is fit in a
 * single characterization run at one battery state; without rescaling to the live bus voltage the
 * same command lands the wheel progressively shorter as the pack drains, and the proportional gain
 * is far too small to make that up — which reads at the driver station as a flywheel that never
 * gets close enough to its setpoint for the feed gate to open.
 */
public class FlywheelVoltageCompensationTest {

  // Live config.yaml values.
  private static final double NOMINAL = 12.0;
  private static final double MAX_SCALE = 1.4;
  private static final double EPS = 1e-9;

  private static double scale(double measuredVolts) {
    return Shooter.voltageCompensationScale(NOMINAL, measuredVolts, MAX_SCALE);
  }

  @Test
  public void testNoCorrectionAtNominal() {
    assertEquals(1.0, scale(NOMINAL), EPS);
  }

  @Test
  public void testSagIsScaledUp() {
    // A 10.8 V pack delivers 90% of the modelled output, so the command must rise by 1/0.9.
    assertEquals(12.0 / 10.8, scale(10.8), EPS);
    assertTrue(scale(11.0) > 1.0, "sag must increase the command");
  }

  @Test
  public void testFreshPackIsScaledDown() {
    // Above nominal the correction has to cut the command, or a fresh battery overshoots the
    // setpoint and the arm window's upper bound blocks feeding from the other side.
    assertEquals(12.0 / 13.2, scale(13.2), EPS);
    assertTrue(scale(13.2) < 1.0, "over-voltage must reduce the command");
  }

  @Test
  public void testDeepSagIsClamped() {
    // 6 V would otherwise ask for 2x the command and rail the motor through a brownout.
    assertEquals(MAX_SCALE, scale(6.0), EPS);
    assertEquals(MAX_SCALE, scale(0.001), EPS);
  }

  @Test
  public void testUnusableReadingFallsBackToUnity() {
    // An unreadable sensor is not evidence of sag. Scaling on it would rail the flywheel on a
    // wiring fault, so an unknown voltage gets the uncompensated command, not the clamp.
    assertEquals(1.0, scale(0.0), EPS);
    assertEquals(1.0, scale(-12.0), EPS);
    assertEquals(1.0, scale(Double.NaN), EPS);
    assertEquals(1.0, scale(Double.POSITIVE_INFINITY), EPS);
  }

  @Test
  public void testUnusableNominalDisablesCompensation() {
    assertEquals(1.0, Shooter.voltageCompensationScale(0.0, 10.0, MAX_SCALE), EPS);
    assertEquals(1.0, Shooter.voltageCompensationScale(Double.NaN, 10.0, MAX_SCALE), EPS);
  }

  @Test
  public void testMissingClampStillCompensates() {
    // A max below 1.0 is not a usable bound; it would forbid even the no-op scale.
    assertEquals(2.0, Shooter.voltageCompensationScale(NOMINAL, 6.0, 0.0), EPS);
  }
}
