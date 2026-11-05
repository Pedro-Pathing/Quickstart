package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/** Rule checks and the AUTO park-route planner in {@link Sentinel}, over generated poses. */
class SentinelProperties {
  /**
   * The planner samples each straight segment about once per inch, so between samples a translating
   * footprint can sweep up to half a step past what was checked. Verifying with a robot this much
   * narrower on each side tolerates exactly that and nothing more.
   */
  private static final double PLANNER_SAMPLING_SLACK_IN = 0.5;

  private final FieldModel blueField = new FieldModel(Alliance.BLUE);
  private final Sentinel red = new Sentinel(Alliance.RED);
  private final Sentinel blue = new Sentinel(blueField);
  private final Sentinel narrowRed;
  private final double size = config.field.size_in;

  SentinelProperties() {
    double width = config.sentinel.robot_width;
    config.sentinel.robot_width = width - 2 * PLANNER_SAMPLING_SLACK_IN;
    try {
      narrowRed = new Sentinel(Alliance.RED);
    } finally {
      config.sentinel.robot_width = width;
    }
  }

  /** G304.A/G402, PARK, LEAVE and the HIVE check must not depend on which alliance asks. */
  @Property(tries = 2000)
  void rulesAreTheSameForBothAlliances(@ForAll("anywhere") Pose redPose) {
    Pose bluePose = blueField.fromRedFrame(redPose);

    assertEquals(red.isFullyOnAllianceSide(redPose), blue.isFullyOnAllianceSide(bluePose), "side");
    assertEquals(red.isParked(redPose), blue.isParked(bluePose), "PARK");
    assertEquals(red.isTouchingWall(redPose), blue.isTouchingWall(bluePose), "wall");
    assertEquals(red.isClearOfHive(redPose), blue.isClearOfHive(bluePose), "HIVE");
  }

  /** The centre-line margin means no footprint can count as fully on both sides at once. */
  @Property(tries = 2000)
  void noPoseIsFullyOnBothSides(@ForAll("anywhere") Pose pose) {
    assertFalse(red.isFullyOnAllianceSide(pose) && blue.isFullyOnAllianceSide(pose));
  }

  /**
   * Whatever route {@link Sentinel#parkRoute} returns is actually safe to drive: it ends at the
   * park pose, and the footprint swept along each straight segment, with the follower's linear
   * heading interpolation, stays in the field, clear of the HIVE frame and on the alliance side at
   * every point, not just at the planner's own samples.
   */
  @Property(tries = 400)
  void parkRouteIsSafeAlongItsWholeSweep(
      @ForAll("legalRedStarts") Pose here, @ForAll("redParkPoses") Pose park) {
    List<Pose> route = red.parkRoute(here, park);
    if (route.isEmpty()) {
      return; // Refusing is always safe; AUTO then stays put.
    }

    assertEquals(park, route.get(route.size() - 1));
    Pose from = here;
    for (Pose to : route) {
      Optional<String> problem = narrowRed.routeProblem(denseSweep(from, to), true);
      Pose segmentStart = from;
      assertTrue(
          problem.isEmpty(),
          () ->
              "segment " + segmentStart + " -> " + to + " " + problem.orElse("") + " in " + route);
      from = to;
    }
  }

  /** Samples at most every 0.05 in of travel and every 0.25 degrees of turn. */
  private static List<Pose> denseSweep(Pose from, Pose to) {
    double turn = AngleUnit.normalizeRadians(to.heading() - from.heading());
    int steps =
        (int)
            Math.max(
                1,
                Math.max(
                    Math.ceil(from.distance(to) / 0.05),
                    Math.ceil(Math.abs(Math.toDegrees(turn)) / 0.25)));
    List<Pose> samples = new ArrayList<>(steps + 1);
    for (int i = 0; i <= steps; i++) {
      double t = (double) i / steps;
      samples.add(
          new Pose(
              from.x() + t * (to.x() - from.x()),
              from.y() + t * (to.y() - from.y()),
              from.heading() + t * turn));
    }
    return samples;
  }

  @Provide
  Arbitrary<Pose> anywhere() {
    Arbitrary<Double> coord = Arbitraries.doubles().between(0, size).ofScale(6);
    Arbitrary<Double> heading =
        Arbitraries.doubles().between(-180, 180).ofScale(6).map(Math::toRadians);
    return Combinators.combine(coord, coord, heading).as(Pose::new);
  }

  /** Where AUTO's park reserve can fire: any legal pose on RED's side. */
  @Provide
  Arbitrary<Pose> legalRedStarts() {
    Arbitrary<Double> x = Arbitraries.doubles().between(9, size / 2).ofScale(6);
    Arbitrary<Double> y = Arbitraries.doubles().between(9, size - 9).ofScale(6);
    Arbitrary<Double> heading =
        Arbitraries.doubles().between(-180, 180).ofScale(6).map(Math::toRadians);
    return Combinators.combine(x, y, heading)
        .as(Pose::new)
        .filter(p -> red.routeProblem(List.of(p), true).isEmpty());
  }

  /** The configured RED park point, approached at any heading. */
  @Provide
  Arbitrary<Pose> redParkPoses() {
    Pose park = MatchProfile.loadMatchProfile(Alliance.RED).parkPose();
    return Arbitraries.doubles()
        .between(-180, 180)
        .ofScale(6)
        .map(Math::toRadians)
        .map(h -> new Pose(park.x(), park.y(), h))
        .filter(p -> red.routeProblem(List.of(p), true).isEmpty());
  }
}
