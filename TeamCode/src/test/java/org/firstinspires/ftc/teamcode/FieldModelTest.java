package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;

/**
 * BIOBUZZ field geometry. Expected values are derived from {@code config.field.*} so a measurement
 * tweak does not break the suite; a few hard-coded manual/CAD figures (about 0.5 in tolerance)
 * anchor the whole thing to the real field.
 */
public class FieldModelTest {

  private static final double EPS = 1e-9;

  private final double size = config.field.size_in;
  private final double centre = size / 2.0;
  private final FieldModel red = new FieldModel(Alliance.RED);
  private final FieldModel blue = new FieldModel(Alliance.BLUE);

  private FieldModel of(Alliance alliance) {
    return alliance == Alliance.RED ? red : blue;
  }

  private static void assertEnvelope(
      String what, double minX, double maxX, double minY, double maxY, Envelope e) {
    assertEquals(minX, e.getMinX(), EPS, what + " minX");
    assertEquals(maxX, e.getMaxX(), EPS, what + " maxX");
    assertEquals(minY, e.getMinY(), EPS, what + " minY");
    assertEquals(maxY, e.getMaxY(), EPS, what + " maxY");
  }

  private static void assertHeading(String what, double expectedRad, double actualRad) {
    assertEquals(0.0, AngleUnit.normalizeRadians(actualRad - expectedRad), 1e-9, what);
  }

  // -------------------------------------------------------------------------------------------
  // CELL openings

  @Test
  public void testRedCellOpeningsMatchTheManual() {
    var h = config.field.hive;
    double x = centre - h.center_to_center_in / 2.0;

    FieldModel.Opening a = red.upOpening(HiveCell.A);
    assertEquals(x, a.x(), EPS);
    assertEquals(centre - h.up_opening_offset_in, a.y(), EPS);
    assertEquals(h.up_opening_height_in, a.heightInches(), EPS);
    assertHeading("RED A faces the audience", -Math.PI / 2, a.facingRad());

    FieldModel.Opening b = red.upOpening(HiveCell.B);
    assertEquals(x, b.x(), EPS);
    assertEquals(centre + h.up_opening_offset_in, b.y(), EPS);
    assertEquals(h.up_opening_height_in, b.heightInches(), EPS);
    assertHeading("RED B faces the rear wall", Math.PI / 2, b.facingRad());

    // Hard-coded anchors from the manual and CAD (see config.yaml): 25.5 in apart, 15.69 in off
    // the HIVE centre line, 59.56 in up.
    assertEquals(58.0, a.x(), 0.5);
    assertEquals(55.06, a.y(), 0.5);
    assertEquals(86.44, b.y(), 0.5);
    assertEquals(59.56, a.heightInches(), 0.5);
  }

  @Test
  public void testBlueCellOpeningsAreRedRotatedAHalfTurn() {
    var h = config.field.hive;
    FieldModel.Opening a = blue.upOpening(HiveCell.A);
    FieldModel.Opening b = blue.upOpening(HiveCell.B);

    // BLUE's CELL A is its rear CELL: it opens toward the rear wall, the mirror of RED's audience
    // A.
    assertEquals(size - (centre - h.center_to_center_in / 2.0), a.x(), EPS);
    assertEquals(size - (centre - h.up_opening_offset_in), a.y(), EPS);
    assertHeading("BLUE A faces the rear wall", Math.PI / 2, a.facingRad());
    assertEquals(a.x(), b.x(), EPS);
    assertEquals(size - (centre + h.up_opening_offset_in), b.y(), EPS);
    assertHeading("BLUE B faces the audience", -Math.PI / 2, b.facingRad());

    assertEquals(83.5, a.x(), 0.5);
    assertEquals(86.44, a.y(), 0.5);
    assertEquals(55.06, b.y(), 0.5);
    assertEquals(h.up_opening_height_in, a.heightInches(), EPS);
  }

  @Test
  public void testEveryBlueGeometryIsTheHalfTurnOfRed() {
    for (HiveCell cell : HiveCell.values()) {
      for (boolean upward : new boolean[] {true, false}) {
        FieldModel.Opening r = red.opening(cell, upward);
        FieldModel.Opening b = blue.opening(cell, upward);
        String what = cell + (upward ? " up" : " down");
        assertEquals(size - r.x(), b.x(), EPS, what + " x");
        assertEquals(size - r.y(), b.y(), EPS, what + " y");
        assertEquals(r.heightInches(), b.heightInches(), EPS, what + " height");
        assertHeading(what + " facing", r.facingRad() + Math.PI, b.facingRad());
      }
    }
    Coordinate rp = red.hivePivot();
    Coordinate bp = blue.hivePivot();
    assertEquals(size - rp.x, bp.x, EPS);
    assertEquals(size - rp.y, bp.y, EPS);
  }

  @Test
  public void testDownOpeningsUseTheLowerOffsetAndHeight() {
    var h = config.field.hive;
    for (Alliance alliance : Alliance.values()) {
      FieldModel f = of(alliance);
      for (HiveCell cell : HiveCell.values()) {
        FieldModel.Opening up = f.upOpening(cell);
        FieldModel.Opening down = f.downOpening(cell);
        String what = alliance + " " + cell;
        assertEquals(h.down_opening_height_in, down.heightInches(), EPS, what);
        assertEquals(h.up_opening_height_in, up.heightInches(), EPS, what);
        assertTrue(down.heightInches() < up.heightInches(), what + ": a down opening is lower");
        assertEquals(up.x(), down.x(), EPS, what + " x unchanged");
        // Down openings sit farther from the centre line (21.36 vs 15.69 in) on the same side.
        assertEquals(h.down_opening_offset_in, Math.abs(down.y() - centre), EPS, what + " offset");
        assertEquals(h.up_opening_offset_in, Math.abs(up.y() - centre), EPS);
        assertEquals(Math.signum(up.y() - centre), Math.signum(down.y() - centre), 0.0);
        assertHeading(what + " facing", up.facingRad(), down.facingRad());
        // opening(cell, upward) picks the right one.
        assertEquals(up.y(), f.opening(cell, true).y(), EPS);
        assertEquals(down.y(), f.opening(cell, false).y(), EPS);
      }
    }
    assertEquals(21.36, h.down_opening_offset_in, 0.5);
    assertEquals(38.17, h.down_opening_height_in, 0.5);
  }

  @Test
  public void testTheTwoCellsOfOneHiveAreOnOppositeSidesOfTheCentreLine() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel f = of(alliance);
      FieldModel.Opening a = f.upOpening(HiveCell.A);
      FieldModel.Opening b = f.upOpening(HiveCell.B);
      assertEquals(centre, (a.y() + b.y()) / 2.0, EPS, alliance + " centre of the pair");
      assertEquals(a.x(), b.x(), EPS, alliance + " same x");
      // They open away from each other.
      assertHeading(
          alliance + " A and B face opposite ways", a.facingRad() + Math.PI, b.facingRad());
      // The pivot sits at the CELLS' x, on the centre line, on our side of the field.
      Coordinate pivot = f.hivePivot();
      assertEquals(a.x(), pivot.x, EPS);
      assertEquals(centre, pivot.y, EPS);
      boolean ownSideIsLowX = alliance == Alliance.RED;
      assertEquals(ownSideIsLowX, pivot.x < centre);
    }
  }

  @Test
  public void testTargetCarriesOpeningAndEpoch() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel f = of(alliance);
      for (HiveCell cell : HiveCell.values()) {
        FieldModel.Opening o = f.upOpening(cell);
        CellTarget t = f.target(cell, 42);
        assertEquals(cell, t.cell());
        assertEquals(o.x(), t.x(), 0.0);
        assertEquals(o.y(), t.y(), 0.0);
        assertEquals(o.heightInches(), t.heightInches(), 0.0);
        assertEquals(o.facingRad(), t.facingRad(), 0.0);
        assertEquals(42, t.epoch());
      }
    }
  }

  // -------------------------------------------------------------------------------------------
  // HIVE frame

  @Test
  public void testHiveFrameIsCentredAndIdenticalForBothAlliances() {
    var h = config.field.hive;
    Envelope r = red.hiveFrame();
    Envelope b = blue.hiveFrame();

    assertEnvelope(
        "RED frame",
        centre - h.frame_width_in / 2.0,
        centre + h.frame_width_in / 2.0,
        centre - h.frame_depth_in / 2.0,
        centre + h.frame_depth_in / 2.0,
        r);
    assertEnvelope("BLUE frame", r.getMinX(), r.getMaxX(), r.getMinY(), r.getMaxY(), b);

    assertEquals(49.46, r.getWidth(), 0.5, "manual: 49.46 in wide");
    assertEquals(38.95, r.getHeight(), 0.5, "manual: 38.95 in deep");
    assertEquals(centre, (r.getMinX() + r.getMaxX()) / 2.0, EPS);
    assertEquals(centre, (r.getMinY() + r.getMaxY()) / 2.0, EPS);
  }

  @Test
  public void testFrameBasesAreTheTwoEndsOfTheFrame() {
    for (Alliance alliance : Alliance.values()) {
      Envelope frame = of(alliance).hiveFrame();
      List<Envelope> bases = of(alliance).hiveFrameBases();
      assertEquals(2, bases.size());
      double w = Math.min(config.sentinel.frame_base_width_in, frame.getWidth() / 2.0);
      assertEnvelope(
          "left base",
          frame.getMinX(),
          frame.getMinX() + w,
          frame.getMinY(),
          frame.getMaxY(),
          bases.get(0));
      assertEnvelope(
          "right base",
          frame.getMaxX() - w,
          frame.getMaxX(),
          frame.getMinY(),
          frame.getMaxY(),
          bases.get(1));
      assertFalse(bases.get(0).intersects(bases.get(1)), "the middle is open floor");
      assertTrue(frame.contains(bases.get(0)) && frame.contains(bases.get(1)));
    }
  }

  @Test
  public void testFrameBaseIsClampedToHalfTheFrame() {
    FieldModel.Dimensions d = fixtureDimensions();
    // A "base" wider than the whole frame is two halves, never overlapping.
    FieldModel.Dimensions wide =
        new FieldModel.Dimensions(
            d.sizeIn(),
            d.hiveCenterToCenterIn(),
            d.upOpeningOffsetIn(),
            d.upOpeningHeightIn(),
            d.downOpeningOffsetIn(),
            d.downOpeningHeightIn(),
            d.frameWidthIn(),
            d.frameDepthIn(),
            1000.0,
            d.loadingZoneStartYIn(),
            d.loadingZoneWidthIn(),
            d.loadingZoneDepthIn(),
            d.gardenLengthIn(),
            d.gardenDepthIn(),
            d.flowerWallOffsetIn(),
            d.flowerCornerOffsetIn());
    List<Envelope> bases = new FieldModel(Alliance.RED, wide).hiveFrameBases();
    assertEquals(d.frameWidthIn() / 2.0, bases.get(0).getWidth(), EPS);
    assertEquals(d.frameWidthIn() / 2.0, bases.get(1).getWidth(), EPS);
  }

  // -------------------------------------------------------------------------------------------
  // Zones

  @Test
  public void testLoadingZonePerAlliance() {
    var lz = config.field.loading_zone;
    assertEnvelope(
        "RED loading zone",
        0.0,
        lz.depth_in,
        lz.start_y_in,
        lz.start_y_in + lz.width_in,
        red.loadingZone());
    assertEnvelope(
        "BLUE loading zone",
        size - lz.depth_in,
        size,
        size - (lz.start_y_in + lz.width_in),
        size - lz.start_y_in,
        blue.loadingZone());

    // CAD anchors: 11 in deep, about 22.7 in along the wall, on the rear half for RED.
    assertEquals(11.0, red.loadingZone().getWidth(), 0.5);
    assertEquals(22.69, red.loadingZone().getHeight(), 0.5);
    assertTrue(red.loadingZone().getMinY() > centre, "RED's zone is on the rear half");
    assertTrue(blue.loadingZone().getMaxY() < centre, "BLUE's zone is on the audience half");
  }

  @Test
  public void testGardenPerAlliance() {
    var g = config.field.garden;
    assertEnvelope("RED garden", 0.0, g.length_in, 0.0, g.depth_in, red.garden());
    assertEnvelope("BLUE garden", size - g.length_in, size, size - g.depth_in, size, blue.garden());
    assertEquals(22.69, red.garden().getWidth(), 0.5);
    assertEquals(2.0, red.garden().getHeight(), 0.5);
  }

  @Test
  public void testAllianceSideAndFieldBounds() {
    assertEnvelope("RED side", 0.0, centre, 0.0, size, red.allianceSide());
    assertEnvelope("BLUE side", centre, size, 0.0, size, blue.allianceSide());
    assertEnvelope("bounds", 0.0, size, 0.0, size, red.fieldBounds());
    assertEnvelope("bounds", 0.0, size, 0.0, size, blue.fieldBounds());
    // The two sides tile the field exactly.
    Envelope union = new Envelope(red.allianceSide());
    union.expandToInclude(blue.allianceSide());
    assertEnvelope("union", 0.0, size, 0.0, size, union);
  }

  @Test
  public void testEveryZoneLiesInsideTheFieldAndOnItsOwnSide() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel f = of(alliance);
      for (Envelope zone : List.of(f.loadingZone(), f.garden())) {
        assertTrue(f.fieldBounds().contains(zone), alliance + " zone inside the field");
        assertTrue(f.allianceSide().contains(zone), alliance + " zone on its own side");
        assertFalse(
            of(other(alliance)).allianceSide().covers(zone),
            "and not on the other alliance's side");
      }
      assertTrue(f.allianceSide().covers(f.hivePivot()), "pivot is on its own side");
    }
  }

  private static Alliance other(Alliance a) {
    return a == Alliance.RED ? Alliance.BLUE : Alliance.RED;
  }

  @Test
  public void testAllianceSideFlowers() {
    var fl = config.field.flower;
    List<Coordinate> r = red.allianceSideFlowers();
    List<Coordinate> b = blue.allianceSideFlowers();
    assertEquals(2, r.size());
    assertEquals(2, b.size());

    // Side-wall FLOWER first, then the rear-wall one.
    assertEquals(fl.wall_offset_in, r.get(0).x, EPS);
    assertEquals(fl.corner_offset_in, r.get(0).y, EPS);
    assertEquals(fl.corner_offset_in, r.get(1).x, EPS);
    assertEquals(size - fl.wall_offset_in, r.get(1).y, EPS);

    for (int i = 0; i < 2; i++) {
      assertEquals(size - r.get(i).x, b.get(i).x, EPS);
      assertEquals(size - r.get(i).y, b.get(i).y, EPS);
    }
    assertEquals(47.36, r.get(0).y, 0.5);
    assertTrue(red.allianceSide().contains(r.get(0)) && red.allianceSide().contains(r.get(1)));
    assertTrue(blue.allianceSide().contains(b.get(0)) && blue.allianceSide().contains(b.get(1)));
  }

  // -------------------------------------------------------------------------------------------
  // Frame transform

  @Test
  public void testFromRedFrameIsIdentityForRedAndAHalfTurnForBlue() {
    Pose p = new Pose(58.0, 9.0, Math.PI / 2);
    Pose same = red.fromRedFrame(p);
    assertEquals(58.0, same.x(), EPS);
    assertEquals(9.0, same.y(), EPS);
    assertHeading("RED heading", Math.PI / 2, same.heading());

    Pose turned = blue.fromRedFrame(p);
    assertEquals(size - 58.0, turned.x(), EPS);
    assertEquals(size - 9.0, turned.y(), EPS);
    assertHeading("BLUE heading", 3 * Math.PI / 2, turned.heading());
    assertEquals(83.5, turned.x(), 0.5);
    assertEquals(132.5, turned.y(), 0.5);
  }

  @Test
  public void testFromRedFrameIsItsOwnInverse() {
    for (Pose p :
        new Pose[] {
          new Pose(1, 2, 0.3), new Pose(70.75, 70.75, -2.0), new Pose(140, 100, Math.PI)
        }) {
      Pose back = blue.fromRedFrame(blue.fromRedFrame(p));
      assertEquals(p.x(), back.x(), 1e-9);
      assertEquals(p.y(), back.y(), 1e-9);
      assertHeading("round trip", p.heading(), back.heading());
    }
    // The field centre is the fixed point.
    Pose centrePose = blue.fromRedFrame(new Pose(centre, centre, 0.0));
    assertEquals(centre, centrePose.x(), EPS);
    assertEquals(centre, centrePose.y(), EPS);
  }

  // -------------------------------------------------------------------------------------------
  // AprilTags

  @Test
  public void testClusterOwnerFromSdkNames() {
    assertOwner(FieldModel.clusterOwner("RED AUDIENCE"), Alliance.RED, HiveCell.A);
    assertOwner(FieldModel.clusterOwner("RED SCORING"), Alliance.RED, HiveCell.B);
    assertOwner(FieldModel.clusterOwner("BLUE AUDIENCE"), Alliance.BLUE, HiveCell.B);
    assertOwner(FieldModel.clusterOwner("BLUE SCORING"), Alliance.BLUE, HiveCell.A);
    // Case-insensitive, and tolerant of the "REAR" spelling of the non-audience side.
    assertOwner(FieldModel.clusterOwner("blue scoring"), Alliance.BLUE, HiveCell.A);
    assertOwner(FieldModel.clusterOwner("Red Rear"), Alliance.RED, HiveCell.B);
  }

  @Test
  public void testClusterOwnerRejectsAnythingAmbiguous() {
    assertTrue(FieldModel.clusterOwner(null).isEmpty());
    assertTrue(FieldModel.clusterOwner("").isEmpty());
    assertTrue(FieldModel.clusterOwner("GREEN AUDIENCE").isEmpty());
    assertTrue(FieldModel.clusterOwner("RED").isEmpty());
    assertTrue(FieldModel.clusterOwner("AUDIENCE").isEmpty());
    assertTrue(FieldModel.clusterOwner("RED BLUE AUDIENCE").isEmpty(), "both alliances named");
    assertTrue(FieldModel.clusterOwner("RED AUDIENCE SCORING").isEmpty(), "both sides named");
    assertTrue(FieldModel.clusterOwner("DECODE OBELISK").isEmpty());
  }

  @Test
  public void testTagOwnerCoversExactlyTheSixteenCellTags() {
    for (int id = 30; id <= 33; id++) {
      assertOwner(FieldModel.tagOwner(id), Alliance.RED, HiveCell.B);
    }
    for (int id = 34; id <= 37; id++) {
      assertOwner(FieldModel.tagOwner(id), Alliance.RED, HiveCell.A);
    }
    for (int id = 38; id <= 41; id++) {
      assertOwner(FieldModel.tagOwner(id), Alliance.BLUE, HiveCell.B);
    }
    for (int id = 42; id <= 45; id++) {
      assertOwner(FieldModel.tagOwner(id), Alliance.BLUE, HiveCell.A);
    }
    for (int id : new int[] {-1, 0, 20, 21, 22, 23, 29, 46, 47, 100, Integer.MAX_VALUE}) {
      assertTrue(FieldModel.tagOwner(id).isEmpty(), "tag " + id + " is not a CELL tag");
    }
  }

  @Test
  public void testClusterFirstIdIsConsistentWithTagOwner() {
    int seen = 0;
    for (Alliance alliance : Alliance.values()) {
      for (HiveCell cell : HiveCell.values()) {
        int first = FieldModel.clusterFirstId(alliance, cell);
        for (int i = 0; i < 4; i++) {
          assertOwner(FieldModel.tagOwner(first + i), alliance, cell);
          seen++;
        }
        assertNotEquals(
            Optional.of(new FieldModel.TagOwner(alliance, cell)),
            FieldModel.tagOwner(first - 1),
            "the tag before a cluster belongs to someone else (or no one)");
      }
    }
    assertEquals(16, seen, "four clusters of four tags");
    assertEquals(34, FieldModel.clusterFirstId(Alliance.RED, HiveCell.A));
    assertEquals(30, FieldModel.clusterFirstId(Alliance.RED, HiveCell.B));
    assertEquals(42, FieldModel.clusterFirstId(Alliance.BLUE, HiveCell.A));
    assertEquals(38, FieldModel.clusterFirstId(Alliance.BLUE, HiveCell.B));
  }

  /** The SDK names and the tag IDs must name the same CELL, or vision and the manual disagree. */
  @Test
  public void testClusterNamesAgreeWithTagIds() {
    String[][] names = {
      {"RED AUDIENCE", "34"}, {"RED SCORING", "30"}, {"BLUE AUDIENCE", "38"}, {"BLUE SCORING", "42"}
    };
    for (String[] n : names) {
      assertEquals(
          FieldModel.tagOwner(Integer.parseInt(n[1])), FieldModel.clusterOwner(n[0]), n[0]);
    }
  }

  private static void assertOwner(
      Optional<FieldModel.TagOwner> owner, Alliance alliance, HiveCell cell) {
    assertTrue(owner.isPresent(), "expected an owner");
    assertEquals(alliance, owner.get().alliance());
    assertEquals(cell, owner.get().cell());
  }

  // -------------------------------------------------------------------------------------------
  // Custom geometry

  private static FieldModel.Dimensions fixtureDimensions() {
    return new FieldModel.Dimensions(
        100.0, // field size
        20.0, // HIVE centre to centre
        10.0, // up opening offset
        50.0, // up opening height
        15.0, // down opening offset
        30.0, // down opening height
        40.0, // frame width
        30.0, // frame depth
        3.0, // frame base width
        60.0, // loading zone start y
        20.0, // loading zone width
        8.0, // loading zone depth
        18.0, // garden length
        2.0, // garden depth
        2.0, // flower wall offset
        40.0 // flower corner offset
        );
  }

  @Test
  public void testCustomDimensionsDriveEveryDerivedValue() {
    FieldModel.Dimensions d = fixtureDimensions();
    FieldModel r = new FieldModel(Alliance.RED, d);
    FieldModel b = new FieldModel(Alliance.BLUE, d);

    assertEquals(50.0, d.center(), 0.0);
    assertEquals(100.0, r.size(), 0.0);
    assertEquals(50.0, r.center(), 0.0);
    assertSame(d, r.dimensions());
    assertEquals(Alliance.BLUE, b.alliance());

    // RED A: x = 50 - 10, y = 50 - 10, facing the audience.
    FieldModel.Opening a = r.upOpening(HiveCell.A);
    assertEquals(40.0, a.x(), EPS);
    assertEquals(40.0, a.y(), EPS);
    assertEquals(50.0, a.heightInches(), EPS);
    assertHeading("custom RED A", -Math.PI / 2, a.facingRad());
    FieldModel.Opening bb = r.upOpening(HiveCell.B);
    assertEquals(60.0, bb.y(), EPS);
    assertEquals(35.0, r.downOpening(HiveCell.A).y(), EPS);
    assertEquals(30.0, r.downOpening(HiveCell.A).heightInches(), EPS);

    // BLUE A is the half turn.
    FieldModel.Opening ba = b.upOpening(HiveCell.A);
    assertEquals(60.0, ba.x(), EPS);
    assertEquals(60.0, ba.y(), EPS);

    assertEnvelope("frame", 30.0, 70.0, 35.0, 65.0, r.hiveFrame());
    assertEnvelope("loading", 0.0, 8.0, 60.0, 80.0, r.loadingZone());
    assertEnvelope("BLUE loading", 92.0, 100.0, 20.0, 40.0, b.loadingZone());
    assertEnvelope("garden", 0.0, 18.0, 0.0, 2.0, r.garden());
    assertEnvelope("BLUE garden", 82.0, 100.0, 98.0, 100.0, b.garden());
    assertEnvelope("side", 0.0, 50.0, 0.0, 100.0, r.allianceSide());
    assertEquals(40.0, r.allianceSideFlowers().get(0).y, EPS);
    assertEquals(98.0, r.allianceSideFlowers().get(1).y, EPS);
  }

  @Test
  public void testDefaultConstructorUsesTheConfigDimensions() {
    FieldModel.Dimensions d = FieldModel.Dimensions.fromConfig();
    assertEquals(config.field.size_in, d.sizeIn(), 0.0);
    assertEquals(config.field.hive.up_opening_height_in, d.upOpeningHeightIn(), 0.0);
    assertEquals(config.sentinel.frame_base_width_in, d.frameBaseWidthIn(), 0.0);
    assertEquals(config.field.loading_zone.start_y_in, d.loadingZoneStartYIn(), 0.0);
    assertEquals(d.sizeIn() / 2.0, d.center(), 0.0);
    assertEquals(d, red.dimensions());
  }
}
