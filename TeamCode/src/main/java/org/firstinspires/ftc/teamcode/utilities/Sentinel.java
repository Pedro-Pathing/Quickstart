package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;

/**
 * BIOBUZZ footprint geometry and rule checks for one alliance, using JTS polygons.
 *
 * <p>Per-match instance owned by {@code Robot}: {@code new Sentinel(fieldModel)} fixes the alliance
 * and zones once. It answers:
 *
 * <ul>
 *   <li>What must the drive protect? The HIVE frame ({@link #getProtectedZones()}). G417 makes
 *       ramming it a foul, and Casablanca uses these zones to slow and repel the robot.
 *   <li>Is a pose legal or scoring? Fully on the alliance side (G304.A start, G402 AUTO), PARK in
 *       the LOADING ZONE, LEAVE (not touching the perimeter wall).
 *   <li>Is a whole autonomous path safe? Its swept footprint must stay inside the field, clear of
 *       the HIVE frame, and on the alliance side ({@link #routeProblem}).
 * </ul>
 */
@Configurable
public class Sentinel {
  private static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();

  /** Footprint within this distance (inches) of a wall counts as touching it. */
  public static final double WALL_CONTACT_TOLERANCE_IN = config.sentinel.wall_contact_tolerance_in;

  private final FieldModel field;
  private final double robotWidth;
  private final List<Envelope> protectedZones;

  public Sentinel(Alliance alliance) {
    this(new FieldModel(alliance));
  }

  public Sentinel(FieldModel field) {
    this.field = field;
    this.robotWidth = config.sentinel.robot_width;
    this.protectedZones =
        config.sentinel.protect_full_hive_footprint
            ? List.of(field.hiveFrame())
            : field.hiveFrameBases();
  }

  public FieldModel field() {
    return field;
  }

  public Alliance getAlliance() {
    return field.alliance();
  }

  public double robotWidth() {
    return robotWidth;
  }

  // ---------------------------------------------------------------------------------------------
  // Protected zones (Casablanca)

  /** Floor regions the drive must not run into: the HIVE frame, for both alliances. */
  public List<Envelope> getProtectedZones() {
    return protectedZones;
  }

  public boolean violatesProtectedZone(Coordinate[] footprint) {
    Polygon robot = createJTSPolygon(footprint);
    for (Envelope zone : protectedZones) {
      if (robot.intersects(GEOMETRY_FACTORY.toGeometry(zone))) {
        return true;
      }
    }
    return false;
  }

  /**
   * Whether turning in place from {@code from} to {@code toHeading} (the short way) keeps the
   * footprint clear of every protected zone at every intermediate heading, sampled every degree. A
   * start already inside a zone is allowed, as in {@link #isRotationSafe}.
   */
  public boolean isRotationClear(Pose from, double toHeading) {
    return isRotationSweepClear(from, AngleUnit.normalizeRadians(toHeading - from.heading()));
  }

  private boolean isRotationSweepClear(Pose from, double turn) {
    if (violatesProtectedZone(calculateRobotFootprint(from))) {
      return true;
    }
    for (Pose p : rotationSamples(from, turn)) {
      if (violatesProtectedZone(calculateRobotFootprint(p))) {
        return false;
      }
    }
    return true;
  }

  /**
   * Checks an in-place pose assist against the HIVE, field walls and optional alliance boundary.
   */
  public Optional<String> rotationProblem(
      Pose from, double toHeading, boolean requireAllianceSide) {
    return routeProblem(
        rotationSamples(from, AngleUnit.normalizeRadians(toHeading - from.heading())),
        requireAllianceSide);
  }

  private static List<Pose> rotationSamples(Pose from, double turn) {
    int steps = Math.max(1, (int) Math.ceil(Math.abs(Math.toDegrees(turn))));
    List<Pose> samples = new ArrayList<>(steps + 1);
    for (int k = 0; k <= steps; k++) {
      samples.add(new Pose(from.x(), from.y(), from.heading() + turn * k / steps));
    }
    return samples;
  }

  /** Checks every intermediate heading in the predicted turn, allowing escape from a zone. */
  public boolean isRotationSafe(Pose currentPose, double turnInput, double lookaheadRad) {
    return isRotationSweepClear(currentPose, Math.signum(turnInput) * lookaheadRad);
  }

  // ---------------------------------------------------------------------------------------------
  // BIOBUZZ rules

  /** G304.A / G402: the whole footprint is on this alliance's side of the field. */
  public boolean isFullyOnAllianceSide(Pose pose) {
    Envelope side = new Envelope(field.allianceSide());
    if (field.alliance() == Alliance.RED) {
      side =
          new Envelope(
              side.getMinX(),
              side.getMaxX() - config.sentinel.center_line_margin_in,
              side.getMinY(),
              side.getMaxY());
    } else {
      side =
          new Envelope(
              side.getMinX() + config.sentinel.center_line_margin_in,
              side.getMaxX(),
              side.getMinY(),
              side.getMaxY());
    }
    side.expandBy(config.sentinel.geometry_epsilon_in);
    return side.contains(getRobotBounds(pose));
  }

  /** PARK (manual 10.5.4): the footprint is at least partially in this alliance's LOADING ZONE. */
  public boolean isParked(Pose pose) {
    return createJTSPolygon(calculateRobotFootprint(pose))
        .intersects(GEOMETRY_FACTORY.toGeometry(field.loadingZone()));
  }

  /** The footprint touches (or crosses) a perimeter wall. LEAVE requires this to be false. */
  public boolean isTouchingWall(Pose pose) {
    Envelope inner = new Envelope(field.fieldBounds());
    inner.expandBy(-config.sentinel.wall_contact_tolerance_in);
    return !inner.contains(getRobotBounds(pose));
  }

  /** End-of-AUTO scoring: LEAVE (not touching the wall) and PARK both earned at this pose. */
  public boolean earnsLeaveAndPark(Pose pose) {
    return isParked(pose) && !isTouchingWall(pose);
  }

  /** The footprint is clear of the HIVE frame (whatever {@link #getProtectedZones()} covers). */
  public boolean isClearOfHive(Pose pose) {
    return !violatesProtectedZone(calculateRobotFootprint(pose));
  }

  /**
   * Checks the swept footprint of a path: inside the field, clear of the HIVE frame, and (when
   * {@code requireAllianceSide}) fully on the alliance side, sampled every {@code stepT}.
   *
   * @return a description of the first violation, or empty if the path is clear
   */
  public Optional<String> routeProblem(Path path, double stepT, boolean requireAllianceSide) {
    List<Pose> samples = new ArrayList<>();
    for (double t = 0.0; t < 1.0; t += stepT) {
      samples.add(path.get(t));
    }
    samples.add(path.get(1.0));
    return routeProblem(samples, requireAllianceSide);
  }

  /** {@link #routeProblem(Path, double, boolean)} over already-sampled poses. */
  public Optional<String> routeProblem(List<Pose> samples, boolean requireAllianceSide) {
    Envelope fieldBounds = new Envelope(field.fieldBounds());
    fieldBounds.expandBy(config.sentinel.geometry_epsilon_in);
    for (Pose p : samples) {
      // The position text is built only for a failing sample: formatting it for every sample cost
      // more than the geometry, and the TeleOp park alert re-plans routes during the match.
      if (!fieldBounds.contains(getRobotBounds(p))) {
        return Optional.of("leaves the field at " + where(p));
      }
      if (!isClearOfHive(p)) {
        return Optional.of("hits the HIVE frame at " + where(p));
      }
      if (requireAllianceSide && !isFullyOnAllianceSide(p)) {
        return Optional.of("crosses the centre line (G402) at " + where(p));
      }
    }
    return Optional.empty();
  }

  private static String where(Pose p) {
    return String.format(Locale.ROOT, "(%.1f, %.1f)", p.x(), p.y());
  }

  /**
   * Waypoints from {@code here} to {@code park} whose straight segments keep the swept footprint in
   * the field, clear of the HIVE frame and on the alliance side. Tries the direct line first, then
   * detours through a lane beside the HIVE on the alliance's side. Used when AUTO's park reserve
   * cancels the routine wherever the robot happens to be.
   *
   * @return poses to follow in order, ending at {@code park}; an empty list if nothing checks clear
   */
  public List<Pose> parkRoute(Pose here, Pose park) {
    for (List<Pose> candidate : candidateRoutes(here, park)) {
      if (segmentsClear(here, candidate)) {
        return candidate;
      }
    }
    // Pushed flush against a wall, the footprint cannot turn without its corners crossing the
    // wall. Slide straight off the walls first, keeping the heading, then plan from there.
    double clearance =
        Math.hypot(robotWidth / 2.0, robotWidth / 2.0) + config.sentinel.park_lane_margin_in;
    double size = field.size();
    clearance = Math.min(clearance, size / 2.0);
    Pose offWall =
        new Pose(
            Math.clamp(here.x(), clearance, size - clearance),
            Math.clamp(here.y(), clearance, size - clearance),
            here.heading());
    if (offWall.distance(here) > 0) {
      for (List<Pose> candidate : candidateRoutes(offWall, park)) {
        List<Pose> withSlide = new ArrayList<>();
        withSlide.add(offWall);
        withSlide.addAll(candidate);
        if (segmentsClear(here, withSlide)) {
          return withSlide;
        }
      }
    }
    return List.of();
  }

  /**
   * Direct; via the HIVE-side lane at this y; along the lane to the park y. The last two also come
   * in a variant that reaches the lane before turning, for a robot pushed close to the HIVE frame,
   * where turning in place would sweep a corner into it.
   */
  private List<List<Pose>> candidateRoutes(Pose from, Pose park) {
    double laneRedX =
        field.dimensions().center()
            - field.dimensions().frameWidthIn() / 2.0
            - Math.hypot(robotWidth / 2.0, robotWidth / 2.0)
            - config.sentinel.park_lane_margin_in;
    Pose fromRed = field.fromRedFrame(from);
    Pose parkRed = field.fromRedFrame(park);
    Pose besideFrom = field.fromRedFrame(new Pose(laneRedX, fromRed.y(), parkRed.heading()));
    Pose besidePark = field.fromRedFrame(new Pose(laneRedX, parkRed.y(), parkRed.heading()));
    Pose laneBeforeTurning = field.fromRedFrame(new Pose(laneRedX, fromRed.y(), fromRed.heading()));
    return List.of(
        List.of(park),
        List.of(besideFrom, park),
        List.of(besideFrom, besidePark, park),
        List.of(laneBeforeTurning, park),
        List.of(laneBeforeTurning, besidePark, park));
  }

  /** Inches between the HIVE-side lane used by {@link #parkRoute} and the robot footprint. */
  public static final double PARK_LANE_MARGIN_IN = config.sentinel.park_lane_margin_in;

  private boolean segmentsClear(Pose start, List<Pose> waypoints) {
    Pose from = start;
    for (Pose to : waypoints) {
      List<Pose> samples = new ArrayList<>();
      int steps = Math.max(2, (int) Math.ceil(from.distance(to)));
      double turn = AngleUnit.normalizeRadians(to.heading() - from.heading());
      for (int i = 0; i <= steps; i++) {
        double t = (double) i / steps;
        // Same linear heading interpolation PathUtil.pline gives the follower.
        samples.add(
            new Pose(
                from.x() + t * (to.x() - from.x()),
                from.y() + t * (to.y() - from.y()),
                from.heading() + t * turn));
      }
      if (routeProblem(samples, true).isPresent()) {
        return false;
      }
      from = to;
    }
    return true;
  }

  // ---------------------------------------------------------------------------------------------
  // Footprint

  public Envelope getRobotBounds(Pose pose) {
    return getProjectedBounds(calculateRobotFootprint(pose));
  }

  public Coordinate[] calculateRobotFootprint(Pose pose) {
    return calculateFootprint(pose, robotWidth);
  }

  private static Coordinate[] calculateFootprint(Pose pose, double width) {
    double heading = pose.heading();
    double centerX = pose.x();
    double centerY = pose.y();
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double radius = width / 2.0;

    double[] xOffsets = {-radius, radius, radius, -radius};
    double[] yOffsets = {-radius, -radius, radius, radius};
    Coordinate[] corners = new Coordinate[4];
    for (int i = 0; i < 4; i++) {
      double rotatedX = (xOffsets[i] * cos) - (yOffsets[i] * sin);
      double rotatedY = (xOffsets[i] * sin) + (yOffsets[i] * cos);
      corners[i] = new Coordinate(centerX + rotatedX, centerY + rotatedY);
    }
    return corners;
  }

  private static Envelope getProjectedBounds(Coordinate[] footprint) {
    Envelope bounds = new Envelope();
    for (Coordinate p : footprint) {
      bounds.expandToInclude(p);
    }
    return bounds;
  }

  private static Polygon createJTSPolygon(Coordinate[] points) {
    if (points == null || points.length == 0) {
      return GEOMETRY_FACTORY.createPolygon();
    }
    Coordinate[] coords = new Coordinate[points.length + 1];
    for (int i = 0; i < points.length; i++) {
      coords[i] = new Coordinate(points[i].x, points[i].y);
    }
    coords[points.length] = new Coordinate(points[0].x, points[0].y);
    return GEOMETRY_FACTORY.createPolygon(coords);
  }
}
