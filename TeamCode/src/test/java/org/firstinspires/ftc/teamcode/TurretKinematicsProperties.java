package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Assume;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.constraints.DoubleRange;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * The moving-shot feedforward ({@link Turret#pivotVelocity}, {@link Turret#requiredRateDps}) is
 * checked against a numerical derivative of the geometry it claims to differentiate: move the robot
 * a tiny step forward and back in time, recompute where the turret has to point, and difference.
 * The oracle shares no formula with the code under test.
 */
class TurretKinematicsProperties {
  /** Central-difference half step, seconds. Truncation error is O(DT^2), far below tolerance. */
  private static final double DT = 1e-5;

  /** Robot motion with constant field velocity and yaw rate. */
  record Motion(Pose robot, double vx, double vy, double omega) {
    Pose at(double t) {
      return new Pose(robot.x() + vx * t, robot.y() + vy * t, robot.heading() + omega * t);
    }

    Velocity velocity() {
      return new Velocity(vx, vy, omega);
    }
  }

  @Property
  void pivotVelocityIsTheDerivativeOfThePivotPosition(
      @ForAll("motions") Motion m,
      @ForAll @DoubleRange(min = -8, max = 8) double pivotForward,
      @ForAll @DoubleRange(min = -8, max = 8) double pivotLeft) {
    withPivotOffset(
        pivotForward,
        pivotLeft,
        () -> {
          Pose ahead = Turret.pivotPose(m.at(DT));
          Pose behind = Turret.pivotPose(m.at(-DT));
          Velocity v = Turret.pivotVelocity(m.robot(), m.velocity());

          assertEquals((ahead.x() - behind.x()) / (2 * DT), v.vx, 1e-4);
          assertEquals((ahead.y() - behind.y()) / (2 * DT), v.vy, 1e-4);
          assertEquals(m.omega(), v.omega, 0.0);
        });
  }

  /**
   * End to end: the rate {@code ShotController} feeds forward is the rate at which the
   * chassis-relative angle to a fixed target actually changes while the robot drives and turns.
   */
  @Property
  void requiredRateKeepsTheTurretOnAFixedTarget(
      @ForAll("motions") Motion m,
      @ForAll("fieldPoints") double[] target,
      @ForAll @DoubleRange(min = -8, max = 8) double pivotForward,
      @ForAll @DoubleRange(min = -8, max = 8) double pivotLeft) {
    withPivotOffset(
        pivotForward,
        pivotLeft,
        () -> {
          Pose pivot = Turret.pivotPose(m.robot());
          // Close to the pivot the bearing is ill-conditioned and the turret never aims there.
          Assume.that(Math.hypot(target[0] - pivot.x(), target[1] - pivot.y()) > 12.0);

          double change =
              AngleUnit.normalizeRadians(
                  chassisRelativeBearing(m.at(DT), target)
                      - chassisRelativeBearing(m.at(-DT), target));
          double numericDps = Math.toDegrees(change / (2 * DT));

          double rateDps =
              Turret.requiredRateDps(
                  pivot, Turret.pivotVelocity(m.robot(), m.velocity()), target[0], target[1]);

          assertEquals(numericDps, rateDps, 1e-3 + 1e-6 * Math.abs(numericDps));
        });
  }

  /**
   * The calibration OpModes place the robot so its turret axis lands on a measured spot:
   * robotPoseForPivot must exactly undo pivotPose, for any offset and heading, and the axis must
   * sit the offset's length from the robot centre.
   */
  @Property
  void robotPoseForPivotUndoesPivotPose(
      @ForAll("motions") Motion m,
      @ForAll @DoubleRange(min = -8, max = 8) double pivotForward,
      @ForAll @DoubleRange(min = -8, max = 8) double pivotLeft) {
    withPivotOffset(
        pivotForward,
        pivotLeft,
        () -> {
          Pose robot = m.robot();
          Pose pivot = Turret.pivotPose(robot);
          Pose back = Turret.robotPoseForPivot(pivot);

          assertEquals(robot.x(), back.x(), 1e-9);
          assertEquals(robot.y(), back.y(), 1e-9);
          assertEquals(robot.heading(), back.heading(), 1e-12);
          assertEquals(
              Math.hypot(pivotForward, pivotLeft),
              Math.hypot(pivot.x() - robot.x(), pivot.y() - robot.y()),
              1e-9);
          // Forward is along the heading: the offset projects onto it as pivotForward.
          double along =
              (pivot.x() - robot.x()) * Math.cos(robot.heading())
                  + (pivot.y() - robot.y()) * Math.sin(robot.heading());
          assertEquals(pivotForward, along, 1e-9);
        });
  }

  /** Where the turret must point, relative to the chassis, to face {@code target} from here. */
  private static double chassisRelativeBearing(Pose robot, double[] target) {
    Pose pivot = Turret.pivotPose(robot);
    return Math.atan2(target[1] - pivot.y(), target[0] - pivot.x()) - robot.heading();
  }

  private static void withPivotOffset(double forward, double left, Runnable body) {
    double savedForward = config.turret.pivot_forward_in;
    double savedLeft = config.turret.pivot_left_in;
    config.turret.pivot_forward_in = forward;
    config.turret.pivot_left_in = left;
    try {
      body.run();
    } finally {
      config.turret.pivot_forward_in = savedForward;
      config.turret.pivot_left_in = savedLeft;
    }
  }

  @Provide
  Arbitrary<Motion> motions() {
    Arbitrary<Double> coord = Arbitraries.doubles().between(0, config.field.size_in).ofScale(6);
    Arbitrary<Double> heading =
        Arbitraries.doubles().between(-180, 180).ofScale(6).map(Math::toRadians);
    Arbitrary<Double> speed = Arbitraries.doubles().between(-80, 80).ofScale(6);
    Arbitrary<Double> yawRate =
        Arbitraries.doubles().between(-360, 360).ofScale(6).map(Math::toRadians);
    return Combinators.combine(coord, coord, heading, speed, speed, yawRate)
        .as((x, y, h, vx, vy, w) -> new Motion(new Pose(x, y, h), vx, vy, w));
  }

  @Provide
  Arbitrary<double[]> fieldPoints() {
    Arbitrary<Double> coord = Arbitraries.doubles().between(0, config.field.size_in).ofScale(6);
    return Combinators.combine(coord, coord).as((x, y) -> new double[] {x, y});
  }
}
