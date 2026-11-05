package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.List;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

/** The park drive-time estimate behind the TeleOp park alert and the AUTO park-reserve tests. */
public class ParkTimeEstimateTest {

  /**
   * Independent oracle: drive rest-to-rest by stepping time, accelerating at {@code a} up to {@code
   * v} and braking at {@code a} as late as possible.
   */
  private static double simulatedSeconds(double distance, double v, double a) {
    double dt = 1e-5;
    double x = 0;
    double speed = 0;
    double t = 0;
    while (x < distance) {
      double stopping = speed * speed / (2 * a);
      if (distance - x <= stopping) {
        speed = Math.max(speed - a * dt, 1e-9);
      } else {
        speed = Math.min(speed + a * dt, v);
      }
      x += speed * dt;
      t += dt;
    }
    return t;
  }

  @Test
  public void driveTimeMatchesASteppedProfileForShortAndLongLegs() {
    double v = 60.0;
    double a = 40.0;
    // v^2 / a = 90 in: below that the profile is a triangle, above it a trapezoid.
    for (double distance : new double[] {1.0, 20.0, 60.0, 89.0, 91.0, 150.0, 300.0}) {
      assertEquals(
          simulatedSeconds(distance, v, a),
          ParkTimeEstimate.driveSeconds(distance, v, a),
          0.01,
          "distance " + distance);
    }
    assertEquals(0.0, ParkTimeEstimate.driveSeconds(0.0, v, a));
  }

  @Test
  public void noRouteOrALostPoseIsInfinitelyLong() {
    Pose here = new Pose(20, 20, 0);
    assertEquals(Double.POSITIVE_INFINITY, ParkTimeEstimate.seconds(here, List.of()));
    assertEquals(
        Double.POSITIVE_INFINITY,
        ParkTimeEstimate.seconds(new Pose(Double.NaN, 20, 0), List.of(new Pose(30, 20, 0))));
  }

  @Test
  public void eachLegIsTheLongerOfDriveAndTurnPlusASettle() {
    var motion = config.pedro.motion;
    double settle = config.sentinel.park_settle_per_waypoint_sec;
    double turnRate = Math.toRadians(config.sentinel.park_turn_rate_dps);
    Pose here = new Pose(20, 20, 0);

    // Pure turn in place: the turn sets the time.
    Pose turned = new Pose(20, 20, Math.PI / 2);
    assertEquals(
        (Math.PI / 2) / turnRate + settle, ParkTimeEstimate.seconds(here, List.of(turned)), 1e-9);

    // Two straight legs: the sum of both legs, each with its settle.
    Pose a = new Pose(50, 20, 0);
    Pose b = new Pose(50, 60, 0);
    double expected =
        ParkTimeEstimate.driveSeconds(
                30, motion.max_strafe_velocity, motion.natural_forward_deceleration)
            + ParkTimeEstimate.driveSeconds(
                40, motion.max_strafe_velocity, motion.natural_forward_deceleration)
            + 2 * settle;
    assertEquals(expected, ParkTimeEstimate.seconds(here, List.of(a, b)), 1e-9);
  }

  @Test
  public void stoppingUsesTheSlowerDecelerationAndUnknownSpeedIsInfinite() {
    var motion = config.pedro.motion;
    double slower =
        Math.min(motion.natural_forward_deceleration, motion.natural_strafe_deceleration);
    assertEquals(0.0, ParkTimeEstimate.stoppingSeconds(0.0));
    assertEquals(60.0 / slower, ParkTimeEstimate.stoppingSeconds(60.0), 1e-12);
    // Oracle: v = v0 - a t reaches zero at v0 / a; check by stepping.
    double v = 60.0;
    double t = 0;
    while (v > 0) {
      v -= slower * 1e-5;
      t += 1e-5;
    }
    assertEquals(t, ParkTimeEstimate.stoppingSeconds(60.0), 1e-3);
    assertEquals(Double.POSITIVE_INFINITY, ParkTimeEstimate.stoppingSeconds(Double.NaN));
    assertEquals(Double.POSITIVE_INFINITY, ParkTimeEstimate.stoppingSeconds(-1.0));
  }

  @Test
  public void aLongerRouteNeverTakesLess() {
    Pose here = new Pose(20, 20, 0);
    double previous = 0;
    for (double y = 21; y < 120; y += 7) {
      double s = ParkTimeEstimate.seconds(here, List.of(new Pose(20, y, 0)));
      assertTrue(s >= previous, "y " + y);
      previous = s;
    }
  }
}
