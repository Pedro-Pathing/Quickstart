package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Assume;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.lifecycle.AfterProperty;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.locationtech.jts.geom.Envelope;

/**
 * What the TeleOp HIVE protection promises the drivers, with the configured limits and braking
 * model: it leaves the drive alone away from the HIVE, it never pushes toward the HIVE harder than
 * the driver asked, and inside the hard-stop band it never moves toward the HIVE at all. The
 * limiter is isolated (friction compensation, smoothing and heading lock off), since friction
 * compensation deliberately boosts small inputs and would hide what the limiter itself does.
 */
class CasablancaSafetyProperties {
  private final CasablancaHarness harness = new CasablancaHarness();
  private final Sentinel sentinel;
  private final Casablanca casablanca;
  private final Envelope hive;

  CasablancaSafetyProperties() {
    // One frame-sized zone. With the two-base layout a robot between the bases is legitimately
    // repelled from one base toward the other, which is not what "toward the HIVE" means here.
    config.sentinel.protect_full_hive_footprint = true;
    sentinel = new Sentinel(Alliance.RED);
    casablanca = new Casablanca(sentinel);
    CasablancaHarness.isolateZoneProtection();
    hive = sentinel.getProtectedZones().get(0);
  }

  @AfterProperty
  void restore() {
    harness.restore();
  }

  @Property(tries = 1000, maxDiscardRatio = 30)
  void awayFromTheHiveTheDriveIsUntouched(
      @ForAll("anywhere") Pose pose,
      @ForAll("velocity") Vector2D velocity,
      @ForAll @DoubleRange(min = -1, max = 1) double strafe,
      @ForAll @DoubleRange(min = -1, max = 1) double forward) {
    Envelope robot = sentinel.getRobotBounds(pose);
    double blend = Casablanca.laneBlendDistance;
    Assume.that(
        Casablanca.calculateLaneFade(
                    robot.getMinX(), robot.getMaxX(), hive.getMinX(), hive.getMaxX(), blend)
                == 0.0
            && Casablanca.calculateLaneFade(
                    robot.getMinY(), robot.getMaxY(), hive.getMinY(), hive.getMaxY(), blend)
                == 0.0);

    double[] out = casablanca.adjustDriveInput(pose, velocity, 0.0, strafe, forward, 0.0);
    Vector2D asked = Vector2D.cartesian(forward, strafe).rotate(pose.heading());

    assertEquals(asked.x(), out[1], 1e-12);
    assertEquals(asked.y(), out[0], 1e-12);
  }

  @Property(tries = 3000)
  void neverPushesTowardTheHiveHarderThanAsked(
      @ForAll("nearHive") Pose pose,
      @ForAll("velocity") Vector2D velocity,
      @ForAll @DoubleRange(min = -1, max = 1) double strafe,
      @ForAll @DoubleRange(min = -1, max = 1) double forward) {
    double[] out = casablanca.adjustDriveInput(pose, velocity, 0.0, strafe, forward, 0.0);
    Vector2D asked = Vector2D.cartesian(forward, strafe).rotate(pose.heading());
    Envelope robot = sentinel.getRobotBounds(pose);

    assertNoHarderTowardHive(
        "X", robot.getMinX(), robot.getMaxX(), hive.getMinX(), hive.getMaxX(), asked.x(), out[1]);
    assertNoHarderTowardHive(
        "Y", robot.getMinY(), robot.getMaxY(), hive.getMinY(), hive.getMaxY(), asked.y(), out[0]);
  }

  /** Squarely in front of a HIVE face and within the hard stop, nothing moves the robot closer. */
  @Property(tries = 3000)
  void insideTheHardStopNothingMovesTowardTheHive(
      @ForAll("inHardStopBand") Approach approach,
      @ForAll("velocity") Vector2D velocity,
      @ForAll @DoubleRange(min = -1, max = 1) double strafe,
      @ForAll @DoubleRange(min = -1, max = 1) double forward) {
    double[] out =
        casablanca.adjustDriveInput(approach.pose(), velocity, 0.0, strafe, forward, 0.0);
    double towardHive = approach.alongX() ? out[1] : out[0];

    assertTrue(
        towardHive * approach.towardHive() <= 0.0,
        () -> approach + " commanded " + towardHive + " toward the HIVE");
  }

  /**
   * Driving parallel to a HIVE face, even inside the hard-stop band, is the driver's business: the
   * along-face command passes through untouched. (Overlapping the face's lane used to count as
   * being inside the HIVE, which braked and then reversed a robot sliding along it.)
   */
  @Property(tries = 2000)
  void drivingAlongAHiveFaceIsUntouched(
      @ForAll("besideAFace") Approach approach,
      @ForAll @DoubleRange(min = -1, max = 1) double alongFace,
      @ForAll @DoubleRange(min = -60, max = 60) double speedAlongFace) {
    Vector2D fieldAsk =
        approach.alongX() ? Vector2D.cartesian(0, alongFace) : Vector2D.cartesian(alongFace, 0);
    Vector2D stick = fieldAsk.rotate(-approach.pose().heading());
    Vector2D velocity =
        approach.alongX()
            ? Vector2D.cartesian(0, speedAlongFace)
            : Vector2D.cartesian(speedAlongFace, 0);

    double[] out =
        casablanca.adjustDriveInput(approach.pose(), velocity, 0.0, stick.y(), stick.x(), 0.0);

    double commandedAlong = approach.alongX() ? out[0] : out[1];
    assertEquals(alongFace, commandedAlong, 1e-9, approach.toString());
  }

  /**
   * A driver braking or backing away from the HIVE is never weakened, whatever the robot is doing:
   * an away command comes out at least as strong as it went in.
   */
  @Property(tries = 3000)
  void backingAwayFromTheHiveIsNeverWeakened(
      @ForAll("nearHive") Pose pose,
      @ForAll("velocity") Vector2D velocity,
      @ForAll @DoubleRange(min = -1, max = 1) double strafe,
      @ForAll @DoubleRange(min = -1, max = 1) double forward) {
    double[] out = casablanca.adjustDriveInput(pose, velocity, 0.0, strafe, forward, 0.0);
    Vector2D asked = Vector2D.cartesian(forward, strafe).rotate(pose.heading());
    Envelope robot = sentinel.getRobotBounds(pose);

    assertAwayKept(
        robot.getMinX(), robot.getMaxX(), hive.getMinX(), hive.getMaxX(), asked.x(), out[1]);
    assertAwayKept(
        robot.getMinY(), robot.getMaxY(), hive.getMinY(), hive.getMaxY(), asked.y(), out[0]);
  }

  private static void assertAwayKept(
      double robotMin, double robotMax, double hiveMin, double hiveMax, double asked, double out) {
    double away;
    if (robotMax <= hiveMin) {
      away = -1.0;
    } else if (robotMin >= hiveMax) {
      away = 1.0;
    } else {
      return;
    }
    if (asked * away > 0) {
      assertTrue(
          out * away >= asked * away - 1e-12, () -> "asked " + asked + " away, commanded " + out);
    }
  }

  private static void assertNoHarderTowardHive(
      String axis,
      double robotMin,
      double robotMax,
      double hiveMin,
      double hiveMax,
      double asked,
      double commanded) {
    double toward;
    if (robotMax <= hiveMin) {
      toward = 1.0;
    } else if (robotMin >= hiveMax) {
      toward = -1.0;
    } else {
      return; // Overlapping on this axis: "toward" is not defined.
    }
    if (commanded * toward > 0.0) {
      assertTrue(
          asked * toward >= commanded * toward - 1e-12,
          () -> axis + ": asked " + asked + " but commanded " + commanded + " toward the HIVE");
    }
  }

  /**
   * A robot whose footprint overlaps one HIVE face's lane, between 0.01 in and just under the hard
   * stop from that face.
   */
  record Approach(Pose pose, boolean alongX, double towardHive) {}

  @Provide
  Arbitrary<Approach> inHardStopBand() {
    return approaches(0.0);
  }

  /**
   * Like {@link #inHardStopBand()}, but overlapping the face's span by at least 0.5 in: not a
   * corner.
   */
  @Provide
  Arbitrary<Approach> besideAFace() {
    return approaches(0.5);
  }

  private Arbitrary<Approach> approaches(double laneMargin) {
    double halfWidth = sentinel.robotWidth() / 2.0;
    Arbitrary<Double> heading =
        Arbitraries.doubles().between(-180, 180).ofScale(6).map(Math::toRadians);
    double hardStop = Math.min(Casablanca.depthHardStop, Casablanca.sideHardStop);
    Arbitrary<Double> gap = Arbitraries.doubles().between(0.01, hardStop - 0.01).ofScale(6);
    Arbitrary<Double> lane = Arbitraries.doubles().between(0.0, 1.0).ofScale(6);
    Arbitrary<Integer> face = Arbitraries.integers().between(0, 3);
    return Combinators.combine(heading, gap, lane, face)
        .as(
            (h, g, l, f) -> {
              // Half the footprint's axis-aligned extent at this heading.
              double e = halfWidth * (Math.abs(Math.cos(h)) + Math.abs(Math.sin(h)));
              boolean alongX = f < 2;
              double toward = f % 2 == 0 ? 1.0 : -1.0;
              double laneMin = (alongX ? hive.getMinY() : hive.getMinX()) - e + laneMargin;
              double laneMax = (alongX ? hive.getMaxY() : hive.getMaxX()) + e - laneMargin;
              double across = laneMin + l * (laneMax - laneMin);
              double faceAt =
                  alongX
                      ? (toward > 0 ? hive.getMinX() : hive.getMaxX())
                      : (toward > 0 ? hive.getMinY() : hive.getMaxY());
              double along = faceAt - toward * (g + e);
              Pose pose = alongX ? new Pose(along, across, h) : new Pose(across, along, h);
              return new Approach(pose, alongX, toward);
            });
  }

  @Provide
  Arbitrary<Pose> nearHive() {
    double c = config.field.size_in / 2.0;
    Arbitrary<Double> coord = Arbitraries.doubles().between(c - 45, c + 45).ofScale(6);
    Arbitrary<Double> heading =
        Arbitraries.doubles().between(-180, 180).ofScale(6).map(Math::toRadians);
    return Combinators.combine(coord, coord, heading).as(Pose::new);
  }

  @Provide
  Arbitrary<Pose> anywhere() {
    Arbitrary<Double> coord = Arbitraries.doubles().between(0, config.field.size_in).ofScale(6);
    Arbitrary<Double> heading =
        Arbitraries.doubles().between(-180, 180).ofScale(6).map(Math::toRadians);
    return Combinators.combine(coord, coord, heading).as(Pose::new);
  }

  @Provide
  Arbitrary<Vector2D> velocity() {
    Arbitrary<Double> speed = Arbitraries.doubles().between(-60, 60).ofScale(6);
    return Combinators.combine(speed, speed).as(Vector2D::cartesian);
  }
}
