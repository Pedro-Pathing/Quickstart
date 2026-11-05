package org.firstinspires.ftc.teamcode.game;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;

/**
 * BIOBUZZ field geometry for one alliance, in the team's Pedro frame (origin at the audience wall /
 * RED alliance wall corner, +x toward the BLUE wall, +y away from the audience).
 *
 * <p>Everything is defined once in RED's frame and mapped to BLUE by a half-turn about the field
 * centre, because the field is 180-degree rotationally symmetric: RED's LOADING ZONE, GARDEN,
 * alliance-side FLOWERS and initially-upward CELL rotate exactly onto BLUE's. The same transform is
 * what {@link ConfigLoader} applies to {@code m} poses.
 *
 * <p>Sources: Competition Manual TU05 Section 9 (HIVE pivot 43.95 in, HIVE centres 25.5 in apart,
 * CELL opening 53.5-65.6 in, frame 49.46 x 38.95 in, LOADING ZONE 23 x 11 in, GARDEN 23 x 2 in) and
 * the official field CAD for the CELL opening centre offset. A per-match immutable value: build one
 * per {@code MatchProfile}, never share a mutable "current alliance".
 */
public final class FieldModel {

  /** Raw RED-frame dimensions, normally read from {@code config.field}. */
  public record Dimensions(
      double sizeIn,
      double hiveCenterToCenterIn,
      double upOpeningOffsetIn,
      double upOpeningHeightIn,
      double downOpeningOffsetIn,
      double downOpeningHeightIn,
      double frameWidthIn,
      double frameDepthIn,
      double frameBaseWidthIn,
      double loadingZoneStartYIn,
      double loadingZoneWidthIn,
      double loadingZoneDepthIn,
      double gardenLengthIn,
      double gardenDepthIn,
      double flowerWallOffsetIn,
      double flowerCornerOffsetIn) {

    public static Dimensions fromConfig() {
      var f = config.field;
      return new Dimensions(
          f.size_in,
          f.hive.center_to_center_in,
          f.hive.up_opening_offset_in,
          f.hive.up_opening_height_in,
          f.hive.down_opening_offset_in,
          f.hive.down_opening_height_in,
          f.hive.frame_width_in,
          f.hive.frame_depth_in,
          config.sentinel.frame_base_width_in,
          f.loading_zone.start_y_in,
          f.loading_zone.width_in,
          f.loading_zone.depth_in,
          f.garden.length_in,
          f.garden.depth_in,
          f.flower.wall_offset_in,
          f.flower.corner_offset_in);
    }

    public double center() {
      return sizeIn / 2.0;
    }
  }

  /** Field position of a CELL opening plus the heading it faces outward. */
  public record Opening(double x, double y, double heightInches, double facingRad) {}

  /** First AprilTag ID of each CELL's four-tag cluster (manual 9.9). */
  private static final int RED_REAR_FIRST_ID = 30;

  private static final int RED_AUDIENCE_FIRST_ID = 34;
  private static final int BLUE_AUDIENCE_FIRST_ID = 38;
  private static final int BLUE_REAR_FIRST_ID = 42;
  private static final int TAGS_PER_CLUSTER = 4;

  /** Which alliance's HIVE an AprilTag belongs to and which of its CELLS. */
  public record TagOwner(Alliance alliance, HiveCell cell) {}

  private final Alliance alliance;
  private final Dimensions d;

  public FieldModel(Alliance alliance) {
    this(alliance, Dimensions.fromConfig());
  }

  public FieldModel(Alliance alliance, Dimensions dimensions) {
    this.alliance = alliance;
    this.d = dimensions;
  }

  public Alliance alliance() {
    return alliance;
  }

  public Dimensions dimensions() {
    return d;
  }

  public double size() {
    return d.sizeIn();
  }

  public double center() {
    return d.center();
  }

  // ---------------------------------------------------------------------------------------------
  // Alliance transform

  /** Maps a RED-frame pose onto this alliance (identity for RED, half-turn for BLUE). */
  public Pose fromRedFrame(Pose redPose) {
    if (alliance == Alliance.RED) {
      return redPose;
    }
    return PoseFactory.radians()
        .mirrorAroundPoint(d.center(), d.center())
        .of(redPose.x(), redPose.y(), redPose.heading());
  }

  private Coordinate fromRedFrame(double x, double y) {
    if (alliance == Alliance.RED) {
      return new Coordinate(x, y);
    }
    return new Coordinate(d.sizeIn() - x, d.sizeIn() - y);
  }

  private Envelope fromRedFrame(Envelope red) {
    Coordinate a = fromRedFrame(red.getMinX(), red.getMinY());
    Coordinate b = fromRedFrame(red.getMaxX(), red.getMaxY());
    return new Envelope(a, b);
  }

  private double headingFromRedFrame(double redHeadingRad) {
    double heading = alliance == Alliance.RED ? redHeadingRad : redHeadingRad + Math.PI;
    return AngleUnit.normalizeRadians(heading);
  }

  // ---------------------------------------------------------------------------------------------
  // HIVE

  /** This alliance's HIVE pivot, projected onto the TILES. */
  public Coordinate hivePivot() {
    return fromRedFrame(d.center() - d.hiveCenterToCenterIn() / 2.0, d.center());
  }

  /**
   * Opening centre of {@code cell} while it is upward-facing: the SDK cluster origin and the point
   * the shooter aims at. RED's CELL A opens toward the audience (heading -90 degrees).
   */
  public Opening upOpening(HiveCell cell) {
    return opening(cell, d.upOpeningOffsetIn(), d.upOpeningHeightIn());
  }

  /** Opening centre of {@code cell} while it is downward-facing. Never a legal shot target. */
  public Opening downOpening(HiveCell cell) {
    return opening(cell, d.downOpeningOffsetIn(), d.downOpeningHeightIn());
  }

  /** Where {@code cell}'s opening is for the given HIVE state. */
  public Opening opening(HiveCell cell, boolean upward) {
    return upward ? upOpening(cell) : downOpening(cell);
  }

  private Opening opening(HiveCell cell, double offset, double height) {
    double redX = d.center() - d.hiveCenterToCenterIn() / 2.0;
    double sign = cell == HiveCell.A ? -1.0 : 1.0;
    Coordinate c = fromRedFrame(redX, d.center() + sign * offset);
    return new Opening(c.x, c.y, height, headingFromRedFrame(sign * Math.PI / 2.0));
  }

  /** Shot target for {@code cell} while it is upward-facing, tagged with the tracker epoch. */
  public CellTarget target(HiveCell cell, int epoch) {
    Opening o = upOpening(cell);
    return new CellTarget(cell, o.x(), o.y(), o.heightInches(), o.facingRad(), epoch);
  }

  /**
   * Floor footprint of the HIVE frame base. Both HIVES share one frame, so this is the same for
   * both alliances.
   */
  public Envelope hiveFrame() {
    double c = d.center();
    return new Envelope(
        c - d.frameWidthIn() / 2.0,
        c + d.frameWidthIn() / 2.0,
        c - d.frameDepthIn() / 2.0,
        c + d.frameDepthIn() / 2.0);
  }

  /** The two triangular frame bases at either end of the HIVE frame. */
  public List<Envelope> hiveFrameBases() {
    Envelope frame = hiveFrame();
    double w = Math.min(d.frameBaseWidthIn(), frame.getWidth() / 2.0);
    return List.of(
        new Envelope(frame.getMinX(), frame.getMinX() + w, frame.getMinY(), frame.getMaxY()),
        new Envelope(frame.getMaxX() - w, frame.getMaxX(), frame.getMinY(), frame.getMaxY()));
  }

  // ---------------------------------------------------------------------------------------------
  // Zones

  /** This alliance's LOADING ZONE (G427 NECTAR entry; PARK target). */
  public Envelope loadingZone() {
    return fromRedFrame(
        new Envelope(
            0.0,
            d.loadingZoneDepthIn(),
            d.loadingZoneStartYIn(),
            d.loadingZoneStartYIn() + d.loadingZoneWidthIn()));
  }

  /** This alliance's GARDEN. Scoring there counts for this alliance whoever placed it. */
  public Envelope garden() {
    return fromRedFrame(new Envelope(0.0, d.gardenLengthIn(), 0.0, d.gardenDepthIn()));
  }

  /** Centres of the two FLOWERS on this alliance's side: side wall first, then rear/front wall. */
  public List<Coordinate> allianceSideFlowers() {
    return List.of(
        fromRedFrame(d.flowerWallOffsetIn(), d.flowerCornerOffsetIn()),
        fromRedFrame(d.flowerCornerOffsetIn(), d.sizeIn() - d.flowerWallOffsetIn()));
  }

  /** This alliance's side of the field (G304.A starting volume, G402 AUTO priority). */
  public Envelope allianceSide() {
    return fromRedFrame(new Envelope(0.0, d.center(), 0.0, d.sizeIn()));
  }

  /** The whole field inside the perimeter walls. */
  public Envelope fieldBounds() {
    return new Envelope(0.0, d.sizeIn(), 0.0, d.sizeIn());
  }

  // ---------------------------------------------------------------------------------------------
  // AprilTags

  /** First tag ID of {@code cell}'s cluster on {@code owner}'s HIVE. */
  public static int clusterFirstId(Alliance owner, HiveCell cell) {
    if (owner == Alliance.RED) {
      return cell == HiveCell.A ? RED_AUDIENCE_FIRST_ID : RED_REAR_FIRST_ID;
    }
    return cell == HiveCell.A ? BLUE_REAR_FIRST_ID : BLUE_AUDIENCE_FIRST_ID;
  }

  /**
   * Which HIVE CELL an SDK cluster belongs to, from its library name ({@code "RED AUDIENCE"},
   * {@code "RED SCORING"}, {@code "BLUE AUDIENCE"}, {@code "BLUE SCORING"}). SDK 12 does not expose
   * a cluster's member IDs, so the name is the only identity a cluster detection carries. "SCORING"
   * is the SDK's name for the CELL on the rear (non-audience) side.
   */
  public static Optional<TagOwner> clusterOwner(String clusterName) {
    if (clusterName == null) {
      return Optional.empty();
    }
    String name = clusterName.toUpperCase(java.util.Locale.ROOT);
    boolean red = name.contains("RED");
    boolean blue = name.contains("BLUE");
    boolean audience = name.contains("AUDIENCE");
    boolean rear = name.contains("SCORING") || name.contains("REAR");
    if (red == blue || audience == rear) {
      return Optional.empty();
    }
    Alliance owner = red ? Alliance.RED : Alliance.BLUE;
    boolean isA = (owner == Alliance.RED) == audience;
    return Optional.of(new TagOwner(owner, isA ? HiveCell.A : HiveCell.B));
  }

  /** Which HIVE CELL a BIOBUZZ AprilTag ID is on, if it is one of the sixteen CELL tags. */
  public static Optional<TagOwner> tagOwner(int id) {
    for (Alliance owner : Alliance.values()) {
      for (HiveCell cell : HiveCell.values()) {
        int first = clusterFirstId(owner, cell);
        if (id >= first && id < first + TAGS_PER_CLUSTER) {
          return Optional.of(new TagOwner(owner, cell));
        }
      }
    }
    return Optional.empty();
  }
}
