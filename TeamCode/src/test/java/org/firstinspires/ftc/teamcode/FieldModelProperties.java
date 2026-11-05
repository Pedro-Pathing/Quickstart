package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.constraints.IntRange;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;

/**
 * Properties of the RED-to-BLUE half-turn that every BLUE field coordinate is derived from. The
 * BIOBUZZ field is rotationally, not mirror, symmetric, so several of these are chosen to fail for
 * a mirror even though a mirror is also distance-preserving and its own inverse.
 */
class FieldModelProperties {
  private static final double EPS = 1e-9;

  private final FieldModel red = new FieldModel(Alliance.RED);
  private final FieldModel blue = new FieldModel(Alliance.BLUE);
  private final double size = red.size();

  @Property
  void halfTurnAppliedTwiceIsTheIdentity(@ForAll("fieldPoses") Pose p) {
    Pose back = blue.fromRedFrame(blue.fromRedFrame(p));

    assertEquals(p.x(), back.x(), EPS);
    assertEquals(p.y(), back.y(), EPS);
    assertEquals(0.0, AngleUnit.normalizeRadians(back.heading() - p.heading()), EPS);
  }

  @Property
  void redFrameIsTheIdentity(@ForAll("fieldPoses") Pose p) {
    Pose same = red.fromRedFrame(p);

    assertEquals(p.x(), same.x(), 0.0);
    assertEquals(p.y(), same.y(), 0.0);
    assertEquals(p.heading(), same.heading(), 0.0);
  }

  /**
   * A mirror also preserves distances, so check orientation too: the signed area of any triangle
   * keeps its sign (and size) under a rotation and flips under a reflection.
   */
  @Property
  void halfTurnPreservesDistancesAndHandedness(
      @ForAll("fieldPoses") Pose a, @ForAll("fieldPoses") Pose b, @ForAll("fieldPoses") Pose c) {
    Pose ra = blue.fromRedFrame(a);
    Pose rb = blue.fromRedFrame(b);
    Pose rc = blue.fromRedFrame(c);

    assertEquals(a.distance(b), ra.distance(rb), 1e-9);
    assertEquals(signedArea(a, b, c), signedArea(ra, rb, rc), 1e-6);
  }

  /**
   * The heading and position transforms must agree: driving forward and then mapping to BLUE lands
   * where mapping to BLUE and then driving forward along the mapped heading does. An off-by-pi (or
   * mirrored) heading transform breaks this even when the points alone look right.
   */
  @Property
  void headingTransformAgreesWithPositionTransform(
      @ForAll("fieldPoses") Pose p, @ForAll @DoubleRange(min = -48, max = 48) double distance) {
    Pose ahead =
        new Pose(
            p.x() + distance * Math.cos(p.heading()),
            p.y() + distance * Math.sin(p.heading()),
            p.heading());

    Pose bp = blue.fromRedFrame(p);
    Pose bAhead = blue.fromRedFrame(ahead);

    assertEquals(bp.x() + distance * Math.cos(bp.heading()), bAhead.x(), 1e-9);
    assertEquals(bp.y() + distance * Math.sin(bp.heading()), bAhead.y(), 1e-9);
  }

  /** Every BLUE zone is RED's zone rotated: membership is preserved point by point. */
  @Property
  void blueZonesAreRedZonesRotated(@ForAll("fieldPoints") Coordinate q) {
    Coordinate rotated = new Coordinate(size - q.x, size - q.y);
    List<Function<FieldModel, Envelope>> zones =
        List.of(
            FieldModel::loadingZone,
            FieldModel::garden,
            FieldModel::allianceSide,
            FieldModel::hiveFrame,
            FieldModel::fieldBounds);

    for (Function<FieldModel, Envelope> zone : zones) {
      assertEquals(zone.apply(red).contains(q), zone.apply(blue).contains(rotated));
    }
  }

  /** Off the centre line, every point belongs to exactly one alliance's side. */
  @Property
  void allianceSidesPartitionTheField(@ForAll("fieldPoints") Coordinate q) {
    boolean onRed = red.allianceSide().contains(q);
    boolean onBlue = blue.allianceSide().contains(q);

    if (q.x == red.center()) {
      assertTrue(onRed && onBlue);
    } else {
      assertTrue(onRed ^ onBlue, () -> "point " + q + " red=" + onRed + " blue=" + onBlue);
    }
    assertEquals(q.x < red.center(), onRed && !onBlue);
  }

  /**
   * Every CELL opening, up or down, for either alliance faces away from its own HIVE pivot, and
   * BLUE's opening is RED's rotated with its facing turned by half a revolution.
   */
  @Property
  void openingsFaceOutwardAndRotateWithTheirHive(
      @ForAll HiveCell cell, @ForAll boolean upward, @ForAll Alliance alliance) {
    FieldModel field = alliance == Alliance.RED ? red : blue;
    FieldModel.Opening o = field.opening(cell, upward);
    Coordinate pivot = field.hivePivot();

    double outward =
        (o.x() - pivot.x) * Math.cos(o.facingRad()) + (o.y() - pivot.y) * Math.sin(o.facingRad());
    assertTrue(outward > 0, () -> alliance + " " + cell + " opening faces its own HIVE");

    FieldModel.Opening r = red.opening(cell, upward);
    FieldModel.Opening b = blue.opening(cell, upward);
    assertEquals(size - r.x(), b.x(), EPS);
    assertEquals(size - r.y(), b.y(), EPS);
    assertEquals(r.heightInches(), b.heightInches(), 0.0);
    assertEquals(0.0, AngleUnit.normalizeRadians(b.facingRad() - r.facingRad() - Math.PI), EPS);
  }

  /**
   * Tag ownership and cluster first IDs are inverses over the sixteen CELL tags, and nothing else.
   */
  @Property
  void tagOwnershipRoundTripsThroughClusterFirstId(@ForAll @IntRange(min = -5, max = 600) int id) {
    Optional<FieldModel.TagOwner> owner = FieldModel.tagOwner(id);

    assertEquals(id >= 30 && id <= 45, owner.isPresent());
    owner.ifPresent(
        o -> {
          int first = FieldModel.clusterFirstId(o.alliance(), o.cell());
          assertTrue(first <= id && id < first + 4);
        });
  }

  @Property
  void clusterNamesAgreeWithTagIds(
      @ForAll Alliance alliance, @ForAll boolean audience, @ForAll boolean lowerCase) {
    String name = alliance.name() + (audience ? " AUDIENCE" : " SCORING");
    if (lowerCase) {
      name = name.toLowerCase(java.util.Locale.ROOT);
    }

    FieldModel.TagOwner byName = FieldModel.clusterOwner(name).orElseThrow();
    // Audience-side tags: RED 34-37, BLUE 38-41. Rear ("SCORING") tags: RED 30-33, BLUE 42-45.
    int firstId = alliance == Alliance.RED ? (audience ? 34 : 30) : (audience ? 38 : 42);
    for (int id = firstId; id < firstId + 4; id++) {
      assertEquals(FieldModel.tagOwner(id).orElseThrow(), byName);
    }
  }

  private static double signedArea(Pose a, Pose b, Pose c) {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
  }

  @Provide
  Arbitrary<Pose> fieldPoses() {
    Arbitrary<Double> coord = Arbitraries.doubles().between(0, size).ofScale(6);
    Arbitrary<Double> heading =
        Arbitraries.doubles().between(-180, 180).ofScale(6).map(Math::toRadians);
    return Combinators.combine(coord, coord, heading).as(Pose::new);
  }

  @Provide
  Arbitrary<Coordinate> fieldPoints() {
    Arbitrary<Double> coord = Arbitraries.doubles().between(0, size).ofScale(6);
    return Combinators.combine(coord, coord).as(Coordinate::new);
  }
}
