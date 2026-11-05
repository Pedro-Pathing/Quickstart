package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.ballistics.FlywheelFeedforwardFit;
import org.junit.jupiter.api.Test;

public class FlywheelFeedforwardFitTest {

  private static List<FlywheelFeedforwardFit.Sample> syntheticSweep(
      double kV, double kS, double... velocities) {
    List<FlywheelFeedforwardFit.Sample> samples = new ArrayList<>();
    for (double v : velocities) {
      samples.add(new FlywheelFeedforwardFit.Sample(kV * v + kS, v));
    }
    return samples;
  }

  @Test
  public void testRecoversExactLineFromNoiselessSamples() {
    double kV = 0.00048;
    double kS = 0.062;
    FlywheelFeedforwardFit.Result fit =
        FlywheelFeedforwardFit.fit(syntheticSweep(kV, kS, 300, 600, 900, 1200, 1500));

    assertEquals(kV, fit.kV(), 1e-9);
    assertEquals(kS, fit.kS(), 1e-9);
    assertEquals(0.0, fit.rmsResidual(), 1e-9);
    assertEquals(0.0, fit.maxResidual(), 1e-9);
  }

  @Test
  public void testFOnSdkScaleAndVMax() {
    double kV = 0.00048;
    double kS = 0.062;
    FlywheelFeedforwardFit.Result fit =
        FlywheelFeedforwardFit.fit(syntheticSweep(kV, kS, 300, 600, 900, 1200, 1500));

    assertEquals(kV * 32767.0, fit.f(), 1e-9);
    // Full power is reached where kV*v + kS == 1.0
    assertEquals((1.0 - kS) / kV, fit.vMax(), 1e-6);
    assertEquals(1.0, fit.predictPower(fit.vMax()), 1e-9);
  }

  @Test
  public void testPredictPowerMatchesModel() {
    FlywheelFeedforwardFit.Result fit =
        FlywheelFeedforwardFit.fit(syntheticSweep(0.0005, 0.05, 400, 800, 1200));
    assertEquals(0.0005 * 870 + 0.05, fit.predictPower(870), 1e-9);
  }

  @Test
  public void testProportionalGainScalesWithRequestedLoopGain() {
    FlywheelFeedforwardFit.Result fit =
        FlywheelFeedforwardFit.fit(syntheticSweep(0.0005, 0.05, 400, 800, 1200));

    // Loop gain = P * plant DC gain = P * (1/kV), so P = loopGain * kV.
    assertEquals(0.30 * 0.0005, fit.proportionalGainFor(0.30), 1e-12);
    assertEquals(1.0, fit.proportionalGainFor(1.0) / fit.kV(), 1e-9);
  }

  /**
   * The failure that motivated this class: the old autotuner anchored its line on exactly two
   * points, so an unsettled full-power sample that reads low tilts the whole feedforward and makes
   * it too hot at every velocity. Fitting the full sweep still degrades under the same bad sample,
   * but strictly less — and unlike the two-point fit it reports a residual, so the bad sample is
   * visible instead of silent.
   */
  @Test
  public void testFullSweepFitBeatsTwoPointFitOnABadSample() {
    double kV = 0.00048;
    double kS = 0.062;
    List<FlywheelFeedforwardFit.Sample> samples =
        syntheticSweep(kV, kS, 300, 600, 900, 1200, 1350, 1425);
    // Corrupt the top-end sample the way an unsettled 4-second step would: velocity reads low.
    samples.set(samples.size() - 1, new FlywheelFeedforwardFit.Sample(1.0, 1380.0));

    FlywheelFeedforwardFit.Result fit = FlywheelFeedforwardFit.fit(samples);

    // What the old two-point method would have concluded from the same corrupted top-end point.
    double twoPointKv = (1.0 - kS) / 1380.0;

    double fullSweepError = Math.abs(fit.kV() - kV);
    double twoPointError = Math.abs(twoPointKv - kV);

    assertTrue(
        fullSweepError < twoPointError,
        "full-sweep fit should be closer to truth than the two-point fit");
    assertTrue(fit.maxResidual() > 1e-3, "bad sample should surface as a residual");
  }

  @Test
  public void testRejectsDegenerateInput() {
    assertThrows(
        IllegalArgumentException.class,
        () -> FlywheelFeedforwardFit.fit(List.of(new FlywheelFeedforwardFit.Sample(0.5, 800))));

    assertThrows(
        IllegalArgumentException.class,
        () ->
            FlywheelFeedforwardFit.fit(
                List.of(
                    new FlywheelFeedforwardFit.Sample(0.5, 800),
                    new FlywheelFeedforwardFit.Sample(0.6, 800))));
  }

  /**
   * Regression guard for the shipped configuration. The autotuner's kS=0.2772 / F=17.1624 pair
   * implies vMax=1380, so at the 870 tick/s setpoint it commands more power than the flywheel needs
   * when the true vMax is 1500 — the open-loop operating point alone lands above the 1.05x gate
   * ceiling, which no proportional gain can rescue.
   */
  @Test
  public void testStaleAutotunerConstantsOvershootGateCeiling() {
    double staleKs = 0.2772;
    double staleF = 17.1624;
    double staleKv = staleF / 32767.0;

    double target = 870.0;
    double command = staleKv * target + staleKs;

    double trueVMax = 1500.0;
    double settledVelocity = trueVMax * (command - staleKs) / (1.0 - staleKs);

    assertTrue(
        settledVelocity > target * 1.05,
        "stale constants settle above the 1.05x flywheel gate ceiling");
  }
}
