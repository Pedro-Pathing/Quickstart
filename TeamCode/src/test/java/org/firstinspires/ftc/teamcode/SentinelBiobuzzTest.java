package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.PathUtil.pcurve;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;

/**
 * Sentinel's BIOBUZZ rule checks: the HIVE frame, the alliance side, PARK/LEAVE and whole-route
 * checks. Both alliances are exercised through the field's half-turn symmetry: whatever holds for a
 * RED pose holds for the same pose rotated about the field centre for BLUE.
 */
public class SentinelBiobuzzTest {

  private static final double SIZE = config.field.size_in;
  private static final double CENTRE = SIZE / 2.0;
  private static final double HALF = config.sentinel.robot_width / 2.0;

  private final Sentinel red = new Sentinel(Alliance.RED);
  private final Sentinel blue = new Sentinel(Alliance.BLUE);

  /** The BLUE twin of a RED pose: a half-turn about the field centre. */
  private static Pose twin(Pose redPose) {
    return new FieldModel(Alliance.BLUE).fromRedFrame(redPose);
  }

  private static Pose twinOf(Pose p) {
    return twin(p);
  }

  // -------------------------------------------------------------------------------------------
  // Construction and zones

  @Test
  public void testConstructionBindsAllianceFieldAndFootprintSize() {
    assertEquals(Alliance.RED, red.getAlliance());
    assertEquals(Alliance.BLUE, blue.getAlliance());
    assertEquals(18.0, config.sentinel.robot_width, 1e-9);
    assertEquals(config.sentinel.robot_width, red.robotWidth(), 0.0);
    assertEquals(Alliance.BLUE, blue.field().alliance());

    FieldModel supplied = new FieldModel(Alliance.BLUE);
    Sentinel viaField = new Sentinel(supplied);
    assertSame(supplied, viaField.field());
    assertEquals(Alliance.BLUE, viaField.getAlliance());
  }

  @Test
  public void testShippedConfigProtectsTheWholeHiveFrameAsOneZone() {
    assertTrue(
        config.sentinel.protect_full_hive_footprint,
        "this suite assumes the shipped full-frame protection");
    for (Sentinel s : List.of(red, blue)) {
      assertEquals(1, s.getProtectedZones().size());
      Envelope zone = s.getProtectedZones().get(0);
      Envelope frame = s.field().hiveFrame();
      assertEquals(frame.getMinX(), zone.getMinX(), 0.0);
      assertEquals(frame.getMaxX(), zone.getMaxX(), 0.0);
      assertEquals(frame.getMinY(), zone.getMinY(), 0.0);
      assertEquals(frame.getMaxY(), zone.getMaxY(), 0.0);
    }
    // The frame is shared, so both alliances protect the same floor.
    assertEquals(red.getProtectedZones().get(0), blue.getProtectedZones().get(0));
  }

  @Test
  public void testProtectingOnlyTheFrameBasesLeavesTheMiddleOpenForADriveUnderRobot() {
    boolean saved = config.sentinel.protect_full_hive_footprint;
    config.sentinel.protect_full_hive_footprint = false;
    try {
      Sentinel bases = new Sentinel(Alliance.RED);
      assertEquals(2, bases.getProtectedZones().size());
      assertEquals(bases.field().hiveFrameBases(), bases.getProtectedZones());

      // Sitting dead centre is legal for a robot that fits under the HIVE...
      assertTrue(bases.isClearOfHive(new Pose(CENTRE, CENTRE, 0)));
      // ...but the triangular bases at either end still are not.
      Envelope frame = bases.field().hiveFrame();
      assertFalse(bases.isClearOfHive(new Pose(frame.getMinX() + 1.0, CENTRE, 0)));
      assertFalse(bases.isClearOfHive(new Pose(frame.getMaxX() - 1.0, CENTRE, 0)));
    } finally {
      config.sentinel.protect_full_hive_footprint = saved;
    }
    assertEquals(1, new Sentinel(Alliance.RED).getProtectedZones().size());
  }

  // -------------------------------------------------------------------------------------------
  // Footprint

  @Test
  public void testFootprintIsAnEighteenInchSquareRotatedByHeading() {
    Coordinate[] c = red.calculateRobotFootprint(new Pose(50, 60, 0));
    assertEquals(4, c.length);
    assertEquals(41, c[0].x, 1e-9);
    assertEquals(51, c[0].y, 1e-9);
    assertEquals(59, c[1].x, 1e-9);
    assertEquals(69, c[2].y, 1e-9);

    // A quarter turn maps the square onto itself.
    Envelope quarter = red.getRobotBounds(new Pose(50, 60, Math.PI / 2));
    assertEquals(41, quarter.getMinX(), 1e-9);
    assertEquals(59, quarter.getMaxX(), 1e-9);
    assertEquals(51, quarter.getMinY(), 1e-9);
    assertEquals(69, quarter.getMaxY(), 1e-9);

    // At 45 degrees the bounding box reaches the half-diagonal.
    double halfDiagonal = HALF * Math.sqrt(2.0);
    Envelope diamond = red.getRobotBounds(new Pose(50, 60, Math.PI / 4));
    assertEquals(50 - halfDiagonal, diamond.getMinX(), 1e-9);
    assertEquals(50 + halfDiagonal, diamond.getMaxX(), 1e-9);
    assertEquals(60 - halfDiagonal, diamond.getMinY(), 1e-9);

    // Every corner is the same distance from the centre, whatever the heading.
    for (double h = -3.0; h <= 3.0; h += 0.5) {
      for (Coordinate corner : red.calculateRobotFootprint(new Pose(50, 60, h))) {
        assertEquals(halfDiagonal, Math.hypot(corner.x - 50, corner.y - 60), 1e-9);
      }
    }
  }

  @Test
  public void testViolatesProtectedZoneFlipsExactlyAtTheFrameEdge() {
    Envelope frame = red.field().hiveFrame();
    double minX = frame.getMinX();

    // Approaching the frame's left edge from outside, axis-aligned.
    Pose clear = new Pose(minX - HALF - 0.01, CENTRE, 0);
    Pose hit = new Pose(minX - HALF + 0.01, CENTRE, 0);
    assertFalse(red.violatesProtectedZone(red.calculateRobotFootprint(clear)));
    assertTrue(red.violatesProtectedZone(red.calculateRobotFootprint(hit)));

    // Rotated 45 degrees the same centre reaches farther, so it already hits.
    Pose sameCentreTurned = new Pose(clear.x(), clear.y(), Math.PI / 4);
    assertTrue(red.violatesProtectedZone(red.calculateRobotFootprint(sameCentreTurned)));
  }

  @Test
  public void testViolationsAreDetectedOnAllFourSidesAndBothAlliances() {
    Envelope f = red.field().hiveFrame();
    double eps = 0.5;
    Pose[] justOutside = {
      new Pose(f.getMinX() - HALF - eps, CENTRE, 0),
      new Pose(f.getMaxX() + HALF + eps, CENTRE, 0),
      new Pose(CENTRE, f.getMinY() - HALF - eps, 0),
      new Pose(CENTRE, f.getMaxY() + HALF + eps, 0)
    };
    for (Pose p : justOutside) {
      for (Sentinel s : List.of(red, blue)) {
        assertFalse(s.violatesProtectedZone(s.calculateRobotFootprint(p)));
        assertTrue(s.isClearOfHive(p));
      }
      Pose inward =
          new Pose(
              p.x() + Math.signum(CENTRE - p.x()) * 2 * eps,
              p.y() + Math.signum(CENTRE - p.y()) * 2 * eps,
              0);
      assertTrue(red.violatesProtectedZone(red.calculateRobotFootprint(inward)));
      assertFalse(red.isClearOfHive(inward));
    }
    assertFalse(red.isClearOfHive(new Pose(CENTRE, CENTRE, 0)));
    assertTrue(red.isClearOfHive(new Pose(10, 10, 0)));
  }

  @Test
  public void testEmptyFootprintDoesNotViolate() {
    assertFalse(red.violatesProtectedZone(new Coordinate[0]));
    assertFalse(red.violatesProtectedZone(null));
  }

  @Test
  public void testRotationSafetyAllowsTurningOutOfAZone() {
    double protectedMaxX = red.getProtectedZones().get(0).getMaxX();
    Pose beside = new Pose(protectedMaxX + HALF + 0.5, CENTRE, 0);

    assertFalse(
        red.isRotationSafe(beside, 1.0, 0.5), "a large turn would sweep a corner into the frame");
    assertFalse(red.isRotationSafe(beside, -1.0, 0.5), "either direction");
    assertTrue(red.isRotationSafe(beside, 1.0, 0.01));
    assertTrue(red.isRotationSafe(beside, 0.0, 10.0), "no turn requested");

    Pose inside = new Pose(CENTRE, CENTRE, 0);
    assertTrue(
        red.isRotationSafe(inside, 1.0, 0.5), "already in the zone: turning is how it gets out");
  }

  // -------------------------------------------------------------------------------------------
  // Alliance side (G304.A / G402)

  @Test
  public void testFullyOnAllianceSideFlipsAtTheCentreLineMinusMargin() {
    double limit = CENTRE - config.sentinel.center_line_margin_in;
    assertTrue(config.sentinel.center_line_margin_in > 0.0);

    // Axis-aligned: the footprint's far edge is at x + HALF.
    assertTrue(red.isFullyOnAllianceSide(new Pose(limit - HALF - 0.01, 60, 0)));
    assertFalse(red.isFullyOnAllianceSide(new Pose(limit - HALF + 0.01, 60, 0)));

    // Turned 45 degrees the same centre sticks out farther.
    double centreX = limit - HALF - 0.5;
    assertTrue(red.isFullyOnAllianceSide(new Pose(centreX, 60, 0)));
    assertFalse(red.isFullyOnAllianceSide(new Pose(centreX, 60, Math.PI / 4)));
  }

  @Test
  public void testAllianceSideIsHalfTurnSymmetric() {
    for (Pose p :
        new Pose[] {
          new Pose(58, 27, Math.PI / 2),
          new Pose(61, 60, 0),
          new Pose(70, 60, 0),
          new Pose(100, 60, 1.0),
          new Pose(30, 100, 4.0)
        }) {
      assertEquals(red.isFullyOnAllianceSide(p), blue.isFullyOnAllianceSide(twinOf(p)));
      assertEquals(
          red.isFullyOnAllianceSide(p),
          blue.isFullyOnAllianceSide(twinOf(p)),
          "the other alliance's side is the complement");
    }
    assertTrue(red.isFullyOnAllianceSide(new Pose(30, 60, 0)));
    assertFalse(blue.isFullyOnAllianceSide(new Pose(30, 60, 0)));
    assertTrue(blue.isFullyOnAllianceSide(new Pose(SIZE - 30, 60, 0)));
    assertFalse(red.isFullyOnAllianceSide(new Pose(SIZE - 30, 60, 0)));
  }

  // -------------------------------------------------------------------------------------------
  // PARK / LEAVE

  @Test
  public void testParkedMeansTheFootprintOverlapsTheLoadingZone() {
    Envelope zone = red.field().loadingZone();
    // The zone's x extent is the wall to the zone depth; a robot against the wall overlaps it.
    Pose inside = new Pose(HALF + 1.0, (zone.getMinY() + zone.getMaxY()) / 2.0, 0);
    assertTrue(red.isParked(inside));

    // Partly in is parked: only the corner reaches into the zone.
    Pose corner = new Pose(zone.getMaxX() + HALF - 1.0, zone.getMinY() - HALF + 1.0, 0);
    assertTrue(red.isParked(corner));

    // Clear of the zone is not.
    assertFalse(red.isParked(new Pose(zone.getMaxX() + HALF + 1.0, zone.getMinY() + 5, 0)));
    assertFalse(red.isParked(new Pose(HALF + 1.0, zone.getMinY() - HALF - 1.0, 0)));
    assertFalse(red.isParked(new Pose(50, 50, 0)));
  }

  @Test
  public void testParkIsAlliancePerSpecific() {
    Pose redParking = new Pose(16, 100, Math.PI / 2);
    assertTrue(red.isParked(redParking));
    assertFalse(blue.isParked(redParking), "RED's zone is not BLUE's");
    assertTrue(blue.isParked(twin(redParking)));
    assertFalse(red.isParked(twin(redParking)));
  }

  @Test
  public void testTouchingWallUsesAHalfInchTolerance() {
    double tol = Sentinel.WALL_CONTACT_TOLERANCE_IN;
    assertEquals(0.5, tol, 0.0);

    // Footprint edge flush against each wall.
    assertTrue(red.isTouchingWall(new Pose(HALF, 60, 0)));
    assertTrue(red.isTouchingWall(new Pose(SIZE - HALF, 60, 0)));
    assertTrue(red.isTouchingWall(new Pose(60, HALF, 0)));
    assertTrue(red.isTouchingWall(new Pose(60, SIZE - HALF, 0)));

    // Within the tolerance still counts as touching; beyond it does not.
    assertTrue(red.isTouchingWall(new Pose(HALF + tol - 0.05, 60, 0)));
    assertFalse(red.isTouchingWall(new Pose(HALF + tol + 0.05, 60, 0)));
    assertFalse(red.isTouchingWall(new Pose(CENTRE, 40, 0)));

    // Rotated, the corners reach the wall sooner.
    assertTrue(red.isTouchingWall(new Pose(HALF + 1.0, 60, Math.PI / 4)));
    // Past the wall counts as touching it too.
    assertTrue(red.isTouchingWall(new Pose(-2, 60, 0)));
  }

  @Test
  public void testEarnsLeaveAndParkNeedsBothParkedAndOffTheWall() {
    Envelope zone = red.field().loadingZone();
    double midY = (zone.getMinY() + zone.getMaxY()) / 2.0;

    // Against the wall in the zone: PARK but not LEAVE.
    Pose onWall = new Pose(HALF, midY, 0);
    assertTrue(red.isParked(onWall));
    assertTrue(red.isTouchingWall(onWall));
    assertFalse(red.earnsLeaveAndPark(onWall));

    // Off the wall and partly in the zone: both.
    Pose good = new Pose(HALF + 3.0, midY, 0);
    assertTrue(red.earnsLeaveAndPark(good));

    // Off the wall but nowhere near the zone: LEAVE only.
    Pose elsewhere = new Pose(HALF + 3.0, 40, 0);
    assertFalse(red.isParked(elsewhere));
    assertFalse(red.earnsLeaveAndPark(elsewhere));

    // Symmetry.
    assertTrue(blue.earnsLeaveAndPark(twin(good)));
    assertFalse(blue.earnsLeaveAndPark(twin(onWall)));
    assertFalse(blue.earnsLeaveAndPark(good));
  }

  // -------------------------------------------------------------------------------------------
  // Route checks

  @Test
  public void testRouteProblemOfAnEmptyOrCleanRouteIsEmpty() {
    assertEquals(Optional.empty(), red.routeProblem(List.of(), true));
    assertEquals(
        Optional.empty(),
        red.routeProblem(
            List.of(new Pose(20, 20, 0), new Pose(30, 30, 1.0), new Pose(40, 20, 0)), true));
  }

  @Test
  public void testRouteProblemNamesWhatWentWrongAndWhere() {
    Optional<String> offField =
        red.routeProblem(List.of(new Pose(20, 20, 0), new Pose(3, 20, 0)), true);
    assertTrue(offField.isPresent());
    assertTrue(offField.get().contains("leaves the field"), offField.get());
    assertTrue(offField.get().contains("(3.0, 20.0)"), offField.get());

    Optional<String> hive =
        red.routeProblem(List.of(new Pose(20, 20, 0), new Pose(CENTRE, CENTRE, 0)), true);
    assertTrue(hive.isPresent());
    assertTrue(hive.get().contains("hits the HIVE frame"), hive.get());

    Optional<String> centre =
        red.routeProblem(List.of(new Pose(20, 20, 0), new Pose(90, 20, 0)), true);
    assertTrue(centre.isPresent());
    assertTrue(centre.get().contains("centre line"), centre.get());
    assertTrue(centre.get().contains("G402"), centre.get());
  }

  @Test
  public void testAllianceSideIsOnlyRequiredWhenAsked() {
    List<Pose> acrossTheLine = List.of(new Pose(20, 20, 0), new Pose(90, 20, 0));
    assertTrue(red.routeProblem(acrossTheLine, true).isPresent());
    assertEquals(
        Optional.empty(),
        red.routeProblem(acrossTheLine, false),
        "TeleOp may cross; only the HIVE and the walls matter then");
  }

  @Test
  public void testRouteProblemReportsTheFirstViolationInOrder() {
    List<Pose> twoProblems =
        List.of(new Pose(20, 20, 0), new Pose(CENTRE, CENTRE, 0), new Pose(1, 1, 0));
    Optional<String> problem = red.routeProblem(twoProblems, true);
    assertTrue(problem.get().contains("HIVE"), problem.get());
    assertFalse(problem.get().contains("leaves the field"), problem.get());
  }

  @Test
  public void testRouteProblemSamplesAPathAlongItsWholeLength() {
    // Straight through the HIVE frame along the centre line.
    var through = pline(new Pose(15, CENTRE, 0), new Pose(60, CENTRE, 0));
    Optional<String> hit = red.routeProblem(through, 0.02, true);
    assertTrue(hit.isPresent());
    assertTrue(hit.get().contains("HIVE"), hit.get());

    // Beside it, parallel, on the audience side.
    var beside = pline(new Pose(15, 30, 0), new Pose(60, 30, 0));
    assertEquals(Optional.empty(), red.routeProblem(beside, 0.02, true));

    // A step coarser than the whole path samples just the end points, so it can miss a middle.
    var tunnel = pline(new Pose(20, 40, 0), new Pose(20, 100, 0));
    assertEquals(Optional.empty(), red.routeProblem(tunnel, 0.02, true));
  }

  @Test
  public void testRouteProblemChecksTheFinalPoseOfAPath() {
    var endsInsideHive = pline(new Pose(20, 30, 0), new Pose(CENTRE, CENTRE, 0));
    assertTrue(red.routeProblem(endsInsideHive, 0.25, true).isPresent());
    var endsOffField = pline(new Pose(20, 30, 0), new Pose(20, 150, 0));
    Optional<String> p = red.routeProblem(endsOffField, 0.25, true);
    assertTrue(p.isPresent());
    assertTrue(p.get().contains("leaves the field"), p.get());
  }

  @Test
  public void testCurvedPathIsCheckedAlongTheCurveNotTheChord() {
    // The chord from (20,40) to (20,100) is fine, but this control point bows the curve to the
    // right, into the HIVE frame.
    var bowed = pcurve(new Pose(20, 40, 0), new Pose(80, 70, 0), new Pose(20, 100, 0));
    Optional<String> p = red.routeProblem(bowed, 0.02, false);
    assertTrue(p.isPresent());
    assertTrue(p.get().contains("HIVE"), p.get());
  }

  @Test
  public void testBluePathChecksAreTheHalfTurnOfRed() {
    var throughRed = pline(new Pose(15, CENTRE, 0), new Pose(60, CENTRE, 0));
    var throughBlue = pline(twin(new Pose(15, CENTRE, 0)), twin(new Pose(60, CENTRE, 0)));
    assertTrue(red.routeProblem(throughRed, 0.02, true).isPresent());
    assertTrue(blue.routeProblem(throughBlue, 0.02, true).isPresent());

    var besideRed = pline(new Pose(15, 30, 0), new Pose(60, 30, 0));
    var besideBlue = pline(twin(new Pose(15, 30, 0)), twin(new Pose(60, 30, 0)));
    assertEquals(Optional.empty(), red.routeProblem(besideRed, 0.02, true));
    assertEquals(Optional.empty(), blue.routeProblem(besideBlue, 0.02, true));

    // And a RED route is a centre-line violation for BLUE.
    assertTrue(blue.routeProblem(besideRed, 0.02, true).isPresent());
  }

  // -------------------------------------------------------------------------------------------
  // Park route

  @Test
  public void testParkRouteGoesDirectWhenTheLineIsClear() {
    Pose here = new Pose(30, 40, Math.PI / 2);
    Pose park = new Pose(16, 99, Math.PI / 2);

    List<Pose> route = red.parkRoute(here, park);

    assertEquals(1, route.size());
    assertSame(park, route.get(0));
  }

  @Test
  public void testParkRouteDetoursThroughTheLaneBesideTheHiveWhenTheLineIsBlocked() {
    // Straight from the right of the HIVE's near end to the far park pose clips the frame.
    Pose here = new Pose(60, 40, Math.PI / 2);
    Pose park = new Pose(30, 110, Math.PI / 2);

    List<Pose> route = red.parkRoute(here, park);

    assertTrue(route.size() >= 2, "a detour has at least one lane waypoint");
    assertSame(park, route.get(route.size() - 1), "always ends on the park pose");
    double laneX =
        CENTRE
            - config.field.hive.frame_width_in / 2.0
            - Math.hypot(config.sentinel.robot_width / 2.0, config.sentinel.robot_width / 2.0)
            - Sentinel.PARK_LANE_MARGIN_IN;
    assertEquals(
        laneX, route.get(0).x(), 1e-9, "first waypoint is in the lane, level with the robot");
    assertEquals(here.y(), route.get(0).y(), 1e-9);
  }

  @Test
  public void testParkRouteForBlueIsTheHalfTurnOfRed() {
    Pose[][] cases = {
      {new Pose(30, 40, Math.PI / 2), new Pose(16, 99, Math.PI / 2)},
      {new Pose(60, 40, Math.PI / 2), new Pose(30, 110, 0.0)},
      {new Pose(58, 27, Math.PI / 2), new Pose(16, 99, Math.PI / 2)},
      {new Pose(40, 20, 0.3), new Pose(16, 119.5, 3 * Math.PI / 2)}
    };
    for (Pose[] c : cases) {
      List<Pose> redRoute = red.parkRoute(c[0], c[1]);
      List<Pose> blueRoute = blue.parkRoute(twin(c[0]), twin(c[1]));

      assertEquals(redRoute.size(), blueRoute.size(), "same number of legs");
      for (int i = 0; i < redRoute.size(); i++) {
        Pose expected = twin(redRoute.get(i));
        Pose actual = blueRoute.get(i);
        String at = "waypoint " + i + " of " + c[0];
        assertEquals(expected.x(), actual.x(), 1e-9, at + " x");
        assertEquals(expected.y(), actual.y(), 1e-9, at + " y");
        assertEquals(
            0.0,
            AngleUnit.normalizeRadians(expected.heading() - actual.heading()),
            1e-9,
            at + " heading");
      }
    }
  }

  @Test
  public void testParkRouteFromClearPosesEndsAtPark() {
    Pose park = new Pose(16, 99, Math.PI / 2);
    for (Pose here :
        new Pose[] {
          new Pose(16, 99, Math.PI / 2),
          new Pose(30, 20, 0),
          new Pose(58, 27, 2.0),
          new Pose(20, 125, 1.0)
        }) {
      List<Pose> route = red.parkRoute(here, park);
      assertNotNull(route);
      assertFalse(route.isEmpty());
      assertSame(park, route.get(route.size() - 1));
    }
  }

  @Test
  public void testParkRouteRefusesAnUnsafeStartingPose() {
    assertTrue(red.parkRoute(new Pose(CENTRE, CENTRE, 0), new Pose(16, 99, 0)).isEmpty());
  }

  @Test
  public void testCentreLineMarginDoesNotShrinkThePerimeterWalls() {
    for (Pose p :
        new Pose[] {new Pose(HALF, 30, 0), new Pose(30, HALF, 0), new Pose(30, SIZE - HALF, 0)}) {
      assertTrue(red.isFullyOnAllianceSide(p));
      assertTrue(blue.isFullyOnAllianceSide(twin(p)));
    }
  }
}
