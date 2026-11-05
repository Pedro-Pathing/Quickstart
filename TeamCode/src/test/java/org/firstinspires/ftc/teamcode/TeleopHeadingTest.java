package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.tests.TeleopHeadingAutotuneOpMode;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.HeadingStepResponse;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

public class TeleopHeadingTest {
  @AfterEach
  public void restore() {
    config.reload();
  }

  @Test
  public void teleopHeadingGainsAndSlewCommandAreIndependentOfAuto() {
    config.casablanca.heading_lock.pid = new PIDFCoefficients(.2, 0, 0, 0);
    config.pedro.heading.pid.p = 99;
    config.casablanca.smoothing.auto_enabled = false;
    var c = new Casablanca(new Sentinel(Alliance.RED));
    c.setTeleopSlewCommand(false).schedule();
    Scheduler.execute();
    assertFalse(c.isTeleopSlewEnabled());
    assertFalse(config.casablanca.smoothing.auto_enabled);
    c.setTargetHeadingLock(.1, true);
    double[] output = c.adjustDriveInput(new Pose(72, 72, 0), Vector2D.zero(), 0, 0, 0, 0, 0);
    assertEquals(.02 + Casablanca.frictionRot + config.casablanca.heading_lock.ks, output[2], 1e-9);
    c.setHeadingPid(new PIDFCoefficients(.3, 0, 0, 0));
    output = c.adjustDriveInput(new Pose(72, 72, 0), Vector2D.zero(), 0, 0, 0, 0, 0);
    assertEquals(.03 + Casablanca.frictionRot + config.casablanca.heading_lock.ks, output[2], 1e-9);
    c.setHeadingPid(null);
    c.setTeleopSlewCommand(true).schedule();
    Scheduler.execute();
    assertTrue(c.isTeleopSlewEnabled());
    assertFalse(config.casablanca.smoothing.auto_enabled);
    assertEquals(99, config.pedro.heading.pid.p, 0);
  }

  @Test
  public void stepResponseRequiresFullObservationAndBothPositionAndRateSettling() {
    for (int sign : new int[] {-1, 1}) {
      var response = new HeadingStepResponse(sign * .2, .03, .1, .05, .2, 1);
      assertEquals(HeadingStepResponse.Status.RUNNING, response.update(sign * .1, .2, .2));
      assertEquals(HeadingStepResponse.Status.RUNNING, response.update(0, 0, .2));
      assertEquals(Double.POSITIVE_INFINITY, response.settlingSeconds(), 0);
      // Leave the settling band again: earlier settling is invalidated.
      response.update(sign * .06, 0, .2);
      response.update(0, .2, .1);
      assertEquals(HeadingStepResponse.Status.PASS, response.update(0, 0, .4));
      assertEquals(1.1, response.settlingSeconds(), 1e-9);
      assertEquals(HeadingStepResponse.Status.PASS, response.update(Double.NaN, 0, .1));
    }
  }

  @Test
  public void overshootTimeoutAndNonFiniteSamplesCannotPass() {
    for (int sign : new int[] {-1, 1}) {
      var response = new HeadingStepResponse(sign * .2, .03, .1, .05, .2, 1);
      assertEquals(HeadingStepResponse.Status.FAIL, response.update(-sign * .06, 0, .1));
      assertEquals(.06, response.overshoot(), 1e-9);
      assertEquals(Double.POSITIVE_INFINITY, response.settlingSeconds(), 0);
    }
    var timeout = new HeadingStepResponse(.2, .03, .1, .05, .2, 1);
    assertEquals(HeadingStepResponse.Status.FAIL, timeout.update(.1, 0, 1));
    for (double[] values :
        new double[][] {
          {Double.NaN, 0, .1}, {0, Double.NaN, .1}, {0, 0, 0}, {0, 0, -.1}, {0, 0, Double.NaN}
        }) {
      var response = new HeadingStepResponse(.2, .03, .1, .05, .2, 1);
      assertEquals(
          HeadingStepResponse.Status.INVALID, response.update(values[0], values[1], values[2]));
    }
  }

  @Test
  public void headingGridAndBoundsAreValidated() {
    assertEquals(1, HeadingStepResponse.gainAt(1, 10, 4, 0), 0);
    assertEquals(4, HeadingStepResponse.gainAt(1, 10, 4, 1), 0);
    assertEquals(10, HeadingStepResponse.gainAt(1, 10, 4, 3), 0);
    assertEquals(1, HeadingStepResponse.gainAt(1, 10, 1, 0), 0);
    for (double[] values :
        new double[][] {
          {Double.NaN, 1, 4, 0},
          {-1, 1, 4, 0},
          {0, Double.NaN, 4, 0},
          {2, 1, 4, 0},
          {0, 1, 0, 0},
          {0, 1, 4, -1},
          {0, 1, 4, 4}
        })
      assertThrows(
          IllegalArgumentException.class,
          () -> HeadingStepResponse.gainAt(values[0], values[1], (int) values[2], (int) values[3]));
    for (int i = 0; i < 6; i++) {
      double[] v = {.2, .03, .1, .05, .2, 1};
      v[i] = Double.NaN;
      assertThrows(
          IllegalArgumentException.class,
          () -> new HeadingStepResponse(v[0], v[1], v[2], v[3], v[4], v[5]));
    }
    assertThrows(
        IllegalArgumentException.class, () -> new HeadingStepResponse(0, .03, .1, .05, .2, 1));
    assertThrows(
        IllegalArgumentException.class, () -> new HeadingStepResponse(.2, .3, .1, .05, .2, 1));
    assertThrows(
        IllegalArgumentException.class, () -> new HeadingStepResponse(.2, .03, .1, .05, 1, 1));
    var settings = config.casablanca.heading_tuner;
    TeleopHeadingAutotuneOpMode.validateSettings(settings);
    settings.heading_tolerance_degrees = .5;
    assertThrows(
        IllegalArgumentException.class,
        () -> TeleopHeadingAutotuneOpMode.validateSettings(settings));
    settings.heading_tolerance_degrees = 2.5;
    settings.maximum_sweep_degrees = Double.NaN;
    assertThrows(
        IllegalArgumentException.class,
        () -> TeleopHeadingAutotuneOpMode.validateSettings(settings));
  }
}
