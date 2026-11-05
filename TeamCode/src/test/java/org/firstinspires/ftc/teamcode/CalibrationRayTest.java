package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ballistics.ShotSolver;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.CalibrationRay;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Test;

/** The ray both shot calibration OpModes walk along from RED's CELL A. */
public class CalibrationRayTest {

  private static final double OFF_AXIS_DEG = -40.0;

  private final FieldModel red = new FieldModel(Alliance.RED);
  private final CalibrationRay ray = CalibrationRay.forCell(red, HiveCell.A, OFF_AXIS_DEG);

  @Test
  public void waypointsAreAtTheRequestedDistanceAndFaceTheOpening() {
    FieldModel.Opening opening = red.upOpening(HiveCell.A);
    for (double d = 18.0; d <= ray.maxDistance(); d += 6.0) {
      Pose p = ray.waypoint(d);
      assertEquals(d, Math.hypot(opening.x() - p.x(), opening.y() - p.y()), 1e-9);
      double bearing = Math.atan2(opening.y() - p.y(), opening.x() - p.x());
      assertEquals(0.0, AngleUnit.normalizeRadians(bearing - p.heading()), 1e-9);
    }
  }

  @Test
  public void everyWaypointIsTheConfiguredAngleOffTheOpeningAxis() {
    CellTarget target = red.target(HiveCell.A, 0);
    for (double d = 18.0; d <= ray.maxDistance(); d += 6.0) {
      Pose p = ray.waypoint(d);
      assertEquals(
          Math.abs(OFF_AXIS_DEG),
          Math.toDegrees(ShotSolver.offAxisRad(p.x(), p.y(), target)),
          1e-6);
    }
    assertTrue(
        Math.abs(OFF_AXIS_DEG) < config.shooter.ballistics.max_off_axis_deg,
        "calibration ray must sit inside the solver's approach gate");
  }

  @Test
  public void maxDistanceStopsAtTheWallClearance() {
    // From CELL A's opening the ray heads 40 deg off straight-at-the-audience, so the audience
    // wall (y = clearance) binds before the RED wall.
    double clearance = CalibrationRay.wallClearanceFor(config.sentinel.robot_width);
    assertEquals(clearance, ray.wallClearance(), 1e-12);
    double expected =
        (red.upOpening(HiveCell.A).y() - clearance) / Math.cos(Math.toRadians(OFF_AXIS_DEG));
    assertEquals(expected, ray.maxDistance(), 1e-9);
    Pose last = ray.waypoint(ray.maxDistance());
    assertEquals(clearance, last.y(), 1e-9);
    assertTrue(last.x() > clearance);
  }

  /**
   * The calibration OpModes start at 24 in. Closer than ~20 in along this ray the footprint meets
   * the HIVE frame, so their first distance must not drop below this.
   */
  @Test
  public void everyCalibrationWaypointIsALegalRobotPose() {
    Sentinel sentinel = new Sentinel(red);
    assertTrue(
        !sentinel.isClearOfHive(ray.waypoint(18.0)),
        "18 in along the ray would hit the HIVE frame");
    for (double d = 24.0; d <= ray.maxDistance(); d += 3.0) {
      Pose p = ray.waypoint(d);
      assertTrue(sentinel.isClearOfHive(p), "clear of the HIVE at " + d);
      assertTrue(!sentinel.isTouchingWall(p), "not touching a wall at " + d);
    }
  }
}
