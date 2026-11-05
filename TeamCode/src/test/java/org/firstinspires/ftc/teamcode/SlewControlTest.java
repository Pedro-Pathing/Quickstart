package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;
import org.firstinspires.ftc.teamcode.pedro.SlewLimitedDrivetrain;
import org.firstinspires.ftc.teamcode.records.SlewProfile;
import org.firstinspires.ftc.teamcode.records.SlewTuning;
import org.firstinspires.ftc.teamcode.records.TiltReading;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.DriveSlewLimiter;
import org.firstinspires.ftc.teamcode.utilities.JumpDetector;
import org.firstinspires.ftc.teamcode.utilities.SlewCalibrationSearch;
import org.firstinspires.ftc.teamcode.utilities.SlewTravelBudget;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class SlewControlTest {
  private static final double EPS = 1e-9;
  private static final SlewProfile LEGACY =
      new SlewProfile(
          new SlewTuning(.22, .55), new SlewTuning(.22, .22), new SlewTuning(.088, .088));

  @AfterEach
  public void restoreConfig() {
    config.reload();
  }

  @Test
  public void preservesCasablancaDirectionalRatesAndDoesNotOvershoot() {
    DriveSlewLimiter limiter = new DriveSlewLimiter();
    powers(limiter.calculate(new DrivePowers(1, -1, 1), .022, LEGACY), .1, -.1, .25);
    powers(limiter.calculate(new DrivePowers(-1, 1, -1), .022, LEGACY), .06, 0, 0);
    powers(limiter.calculate(new DrivePowers(.061, -.001, .001), .022, LEGACY), .061, -.001, .001);
    powers(limiter.calculate(DrivePowers.zero(), 0, LEGACY), .061, -.001, .001);
    limiter.reset();
    powers(limiter.calculate(new DrivePowers(-1, 0, 0), .022, LEGACY), -.04, 0, 0);
    // Reverse braking is the faster direction; forward braking is the slower direction.
    powers(limiter.calculate(DrivePowers.zero(), .022, LEGACY), 0, 0, 0);
    assertEquals(.9, DriveSlewLimiter.limit(1, 0, .1), EPS);
    assertEquals(-.9, DriveSlewLimiter.limit(-1, 0, .1), EPS);
  }

  @Test
  public void invalidInputsFailToZero() {
    DriveSlewLimiter limiter = new DriveSlewLimiter();
    DrivePowers[] invalid = {
      null,
      new DrivePowers(Double.NaN, 0, 0),
      new DrivePowers(0, Double.POSITIVE_INFINITY, 0),
      new DrivePowers(0, 0, Double.NaN)
    };
    for (DrivePowers input : invalid) {
      assertFalse(DriveSlewLimiter.finite(input));
      limiter.reset(new DrivePowers(1, 1, 1));
      powers(limiter.calculate(input, .01, LEGACY), 0, 0, 0);
      limiter.reset(input);
      powers(limiter.output(), 0, 0, 0);
    }
    for (double bad : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -1}) {
      powers(limiter.calculate(new DrivePowers(1, 1, 1), bad, LEGACY), 0, 0, 0);
    }
    powers(limiter.calculate(new DrivePowers(1, 1, 1), .01, null), 0, 0, 0);
  }

  @Test
  public void wrapperPassesNativeAndManualCommandsExactlyAndStopsImmediately() {
    var s = config.casablanca.smoothing;
    s.auto_enabled = true;
    s.forward.increase_time = s.strafe.increase_time = .2;
    s.forward.decrease_time = .4;
    s.strafe.decrease_time = .2;
    s.turn.increase_time = s.turn.decrease_time = .1;
    s.max_loop_dt = .05;
    Drivetrain raw = mock(Drivetrain.class);
    AtomicLong clock = new AtomicLong();
    var wrapper = new SlewLimitedDrivetrain(raw, clock::get);
    DrivePowers target = new DrivePowers(1, 1, 1);
    clock.set(10_000_000);
    wrapper.drive(target, false);
    ArgumentCaptor<DrivePowers> captured = ArgumentCaptor.forClass(DrivePowers.class);
    verify(raw).drive(captured.capture(), eq(false));
    powers(captured.getValue(), .05, .05, .1);
    clearInvocations(raw);
    // Long idle/GC stall must not jump immediately to full power.
    clock.set(10_000_000_000L);
    wrapper.drive(target, false);
    verify(raw).drive(captured.capture(), eq(false));
    powers(captured.getValue(), .3, .3, .6);
    clearInvocations(raw);
    wrapper.drive(target, true);
    verify(raw).drive(same(target), eq(true));
    s.auto_enabled = false;
    DrivePowers reverse = new DrivePowers(-1, -.5, -.2);
    wrapper.drive(reverse, false);
    verify(raw).drive(same(reverse), eq(false));
    wrapper.stop(true);
    verify(raw).stop(true);
    wrapper.stop(false);
    verify(raw).stop(false);
    wrapper.stop();
    verify(raw).stop();
    clearInvocations(raw);
    s.auto_enabled = true;
    clock.addAndGet(10_000_000);
    wrapper.drive(target, false);
    verify(raw).drive(captured.capture(), eq(false));
    powers(captured.getValue(), .05, .05, .1);
    wrapper.drive(new DrivePowers(Double.NaN, 0, 0), true);
    verify(raw).stop(true);
    when(raw.maxScaling(target, reverse)).thenReturn(.123);
    when(raw.interpolateVelocity(1, 2, 3)).thenReturn(4.0);
    when(raw.interpolateAcceleration(1, 2, 3)).thenReturn(5.0);
    Map<String, Object> debug = Map.of("native", true);
    when(raw.debug()).thenReturn(debug);
    assertEquals(.123, wrapper.maxScaling(target, reverse), EPS);
    assertEquals(4, wrapper.interpolateVelocity(1, 2, 3), EPS);
    assertEquals(5, wrapper.interpolateAcceleration(1, 2, 3), EPS);
    assertSame(debug, wrapper.debug());
  }

  @Test
  public void tiltRatesWrapAnglesAndRejectNonAdvancingClock() {
    TiltReading before = new TiltReading(179, -179, 1_000_000_000);
    TiltReading after = new TiltReading(-179, 179, 1_100_000_000);
    assertTrue(before.finite());
    assertEquals(.1, after.secondsSince(before), EPS);
    assertEquals(20, after.pitchRate(before), EPS);
    assertEquals(-20, after.rollRate(before), EPS);
    assertTrue(Double.isNaN(before.pitchRate(before)));
    assertTrue(Double.isNaN(before.rollRate(after)));
    assertFalse(new TiltReading(Double.NaN, 0, 0).finite());
    assertFalse(new TiltReading(0, Double.POSITIVE_INFINITY, 0).finite());
  }

  @Test
  public void jumpDetectionUsesSustainedTiltOrRateAndLatches() {
    JumpDetector d = new JumpDetector(179, 0, 3, 90, .04, 10);
    assertEquals(JumpDetector.Status.CLEAR, d.update(-179, 0, 0, 0, .02));
    assertEquals(JumpDetector.Status.CLEAR, d.update(-177, 0, 0, 0, .02));
    // An isolated bump is not sustained.
    assertEquals(JumpDetector.Status.CLEAR, d.update(179, 0, 0, 0, .02));
    assertEquals(JumpDetector.Status.CLEAR, d.update(179, 0, 0, 100, .02));
    assertEquals(JumpDetector.Status.JUMP, d.update(179, 0, 0, 100, .02));
    assertEquals(JumpDetector.Status.JUMP, d.update(179, 0, 0, 0, .02));
    assertTrue(d.detected());
    assertEquals(4, d.peakTilt(), EPS);
    assertEquals(100, d.peakRate(), EPS);
    assertEquals(JumpDetector.Status.EMERGENCY, d.update(169, 0, 0, 0, .01));
    JumpDetector tilt = new JumpDetector(0, 0, 3, 90, .04, 10);
    assertEquals(JumpDetector.Status.JUMP, tilt.update(3, 4, 0, 0, .04));
    assertEquals(5, tilt.peakTilt(), EPS);
  }

  @Test
  public void detectorRejectsBadSamplesAndThresholds() {
    JumpDetector d = new JumpDetector(0, 0, 3, 90, .04, 10);
    double[][] samples = {
      {Double.NaN, 0, 0, 0, .01},
      {0, Double.NaN, 0, 0, .01},
      {0, 0, Double.NaN, 0, .01},
      {0, 0, 0, Double.NaN, .01},
      {0, 0, 0, 0, Double.NaN},
      {0, 0, 0, 0, -.01}
    };
    for (double[] v : samples)
      assertEquals(JumpDetector.Status.INVALID, d.update(v[0], v[1], v[2], v[3], v[4]));
    assertFalse(d.detected());
    assertEquals(JumpDetector.Status.CLEAR, d.update(0, 0, 0, 0, 0));
    for (int index = 0; index < 6; index++) {
      double[] p = {0, 0, 3, 90, .04, 10};
      p[index] = Double.NaN;
      assertThrows(
          IllegalArgumentException.class,
          () -> new JumpDetector(p[0], p[1], p[2], p[3], p[4], p[5]));
    }
    assertThrows(IllegalArgumentException.class, () -> new JumpDetector(0, 0, 3, 90, .04, 3));
    assertThrows(IllegalArgumentException.class, () -> new JumpDetector(0, 0, 0, 90, .04, 10));
  }

  @Test
  public void contactSearchRemovesOnlyDirectionsThatPass() {
    var search = new SlewCalibrationSearch(new SlewTuning(.2, .4), .1, .8, 2, 12);
    assertNull(search.best());
    search.record(true);
    assertEquals(new SlewTuning(0, .4), search.candidate());
    search.record(true); // Increasing power needs no slew.
    assertEquals(new SlewTuning(0, 0), search.candidate());
    search.record(false); // Decreasing power still lifts a pod without slew.
    assertEquals(new SlewTuning(0, .2), search.candidate());
    search.record(true);
    assertEquals(new SlewTuning(0, .1), search.candidate());
    search.record(false);
    assertTrue(search.done());
    assertEquals(new SlewTuning(0, .2), search.best());
    assertThrows(IllegalStateException.class, () -> search.record(true));
  }

  @Test
  public void searchFindsBaselineAndStopsAtBoundsOrBudget() {
    var search = new SlewCalibrationSearch(new SlewTuning(.2, .2), .1, .8, 2, 12);
    search.record(false);
    assertEquals(new SlewTuning(.4, .4), search.candidate());
    search.record(true);
    search.record(false); // Unrestricted increase fails.
    search.record(true); // .2 increase passes.
    search.record(false); // .1 increase fails.
    assertEquals(new SlewTuning(.2, 0), search.candidate());
    search.record(false); // Unrestricted decrease fails.
    search.record(true); // .2 decrease passes.
    search.record(true); // .1 decrease passes at bound.
    assertTrue(search.done());
    assertEquals(new SlewTuning(.2, .1), search.best());
    var free = new SlewCalibrationSearch(new SlewTuning(0, 0), .1, .8, 2, 12);
    free.record(true);
    assertTrue(free.done());
    var limited = new SlewCalibrationSearch(new SlewTuning(.1, .1), .1, .8, 2, 12);
    limited.record(true);
    limited.record(false);
    limited.record(false);
    assertTrue(limited.done());
    var failure = new SlewCalibrationSearch(new SlewTuning(0, 0), .1, .8, 2, 12);
    while (!failure.done()) failure.record(false);
    assertEquals(SlewCalibrationSearch.Stage.FAILED, failure.stage());
    assertNull(failure.best());
    for (boolean pass : new boolean[] {false, true}) {
      var budget = new SlewCalibrationSearch(new SlewTuning(.2, .2), .1, .8, 2, 1);
      budget.record(pass);
      assertTrue(budget.done());
      assertEquals(pass, budget.best() != null);
    }
  }

  @Test
  public void invalidCalibrationTimesAreRejectedButZeroIsAValidBypass() {
    for (double bad : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
      assertThrows(IllegalArgumentException.class, () -> new SlewTuning(bad, 0));
      assertThrows(IllegalArgumentException.class, () -> new SlewTuning(0, bad));
    }
    assertEquals(0, new SlewTuning(0, 0).longestTime(), 0);
    for (double bad : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
      assertThrows(
          IllegalArgumentException.class,
          () -> new SlewCalibrationSearch(new SlewTuning(.2, .2), bad, .8, 2, 12));
      assertThrows(
          IllegalArgumentException.class,
          () -> new SlewCalibrationSearch(new SlewTuning(.2, .2), .1, bad, 2, 12));
      assertThrows(
          IllegalArgumentException.class,
          () -> new SlewCalibrationSearch(new SlewTuning(.2, .2), .1, .8, bad, 12));
    }
    assertThrows(
        IllegalArgumentException.class,
        () -> new SlewCalibrationSearch(new SlewTuning(.2, .2), .1, .8, 1, 12));
    assertThrows(
        IllegalArgumentException.class,
        () -> new SlewCalibrationSearch(new SlewTuning(.2, .2), .1, .8, 2, 0));
    assertThrows(
        IllegalArgumentException.class,
        () -> new SlewCalibrationSearch(new SlewTuning(1, .2), .1, .8, 2, 12));
  }

  @Test
  public void onlySelectedSignedDirectionsHaveLatency() {
    var profile =
        new SlewProfile(new SlewTuning(0, .5), new SlewTuning(0, 0), new SlewTuning(0, 0));
    var limiter = new DriveSlewLimiter();
    powers(limiter.calculate(new DrivePowers(1, 1, -.8), .01, profile), 1, 1, -.8);
    powers(limiter.calculate(new DrivePowers(-1, -.6, .9), .01, profile), .98, -.6, .9);
    powers(limiter.calculate(new DrivePowers(1, -.6, .9), .01, profile), 1, -.6, .9);
    powers(limiter.calculate(new DrivePowers(1, -.6, .9), 0, profile), 1, -.6, .9);
    for (SlewProfile.Axis axis : SlewProfile.Axis.values()) {
      var replacement = new SlewTuning(.3, .7);
      assertEquals(replacement, profile.with(axis, replacement).axis(axis));
    }
    assertEquals(LEGACY, SlewProfile.configured());
  }

  @Test
  public void travelReserveIncludesFullRampCoastReactionAndMargin() {
    // 20*(.5*.4+.05) + (20*.1+400*.002)/.7 + 4 = 13 inches.
    assertEquals(13, SlewTravelBudget.stoppingRoom(20, .5, .4, .1, .002, .7, .05, 4), EPS);
    assertEquals(13, SlewTravelBudget.stoppingRoom(-20, -.5, .4, .1, .002, .7, .05, 4), EPS);
    assertEquals(4, SlewTravelBudget.stoppingRoom(0, 0, .4, .1, .002, .7, .05, 4), EPS);
    for (int index = 0; index < 8; index++) {
      double[] p = {20, .5, .4, .1, .002, .7, .05, 4};
      p[index] = Double.NaN;
      assertEquals(Double.POSITIVE_INFINITY, room(p), 0);
    }
    for (int index = 2; index < 8; index++) {
      double[] p = {20, .5, .4, .1, .002, .7, .05, 4};
      p[index] = -1;
      assertEquals(Double.POSITIVE_INFINITY, room(p), 0);
    }
    assertEquals(9, room(new double[] {20, .5, 0, .1, .002, .7, .05, 4}), EPS);
    assertEquals(Double.POSITIVE_INFINITY, room(new double[] {20, .5, .4, .1, .002, 0, .05, 4}), 0);
    assertEquals(
        Double.POSITIVE_INFINITY, room(new double[] {20, .5, .4, .1, .002, 1.1, .05, 4}), 0);
  }

  private static double room(double[] p) {
    return SlewTravelBudget.stoppingRoom(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
  }

  private static void powers(DrivePowers actual, double forward, double strafe, double turn) {
    assertEquals(forward, actual.forward(), EPS);
    assertEquals(strafe, actual.strafe(), EPS);
    assertEquals(turn, actual.turn(), EPS);
  }
}
