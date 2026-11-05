package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.HiveVision;
import org.firstinspires.ftc.teamcode.utilities.HiveVision.CameraMount;
import org.firstinspires.ftc.teamcode.utilities.HiveVision.CellSighting;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.math.Vector3D;

/**
 * The pure geometry of HiveVision: camera frame to robot frame, upward classification, and
 * relocalizing from a CELL opening. The camera is pitched up 50 degrees on the real robot, so a
 * mistake in the rotation is a large, silent error in both height (which decides "is this CELL
 * up?") and position.
 */
public class HiveVisionGeometryTest {

  private static final double EPS = 1e-9;

  private static CameraMount mount(
      double fwd, double left, double height, double yawDeg, double pitchDeg) {
    return new CameraMount(fwd, left, height, Math.toRadians(yawDeg), Math.toRadians(pitchDeg));
  }

  private static CellSighting sighting(double fwd, double left, double height) {
    return new CellSighting(Alliance.RED, HiveCell.A, fwd, left, height, 0L, 100);
  }

  private static void assertVec(String what, double[] expected, double[] actual) {
    assertEquals(3, actual.length);
    for (int i = 0; i < 3; i++) {
      assertEquals(expected[i], actual[i], 1e-9, what + "[" + i + "]");
    }
  }

  private static double norm(double[] value) {
    return new Vector3D(value[0], value[1], value[2]).length();
  }

  private static double dot(double[] a, double[] b) {
    return new Vector3D(a[0], a[1], a[2]).dot(new Vector3D(b[0], b[1], b[2]));
  }

  // -------------------------------------------------------------------------------------------
  // cameraToRobot

  @Test
  public void testAZeroOffsetPointIsJustTheMountPosition() {
    CameraMount m = mount(3.54, 4.72, 7.68, 20.0, 50.0);
    assertVec("origin", new double[] {3.54, 4.72, 7.68}, HiveVision.cameraToRobot(0, 0, 0, m));
  }

  @Test
  public void testLevelForwardCameraAxesMapStraightThrough() {
    CameraMount m = mount(0, 0, 0, 0, 0);
    // SDK ftcPose: x right, y out of the lens, z up. Robot frame: forward, left, up.
    assertVec(
        "out of the lens is forward",
        new double[] {10, 0, 0},
        HiveVision.cameraToRobot(0, 10, 0, m));
    assertVec(
        "camera right is robot right (negative left)",
        new double[] {0, -10, 0},
        HiveVision.cameraToRobot(10, 0, 0, m));
    assertVec(
        "camera up is robot up", new double[] {0, 0, 10}, HiveVision.cameraToRobot(0, 0, 10, m));
  }

  @Test
  public void testYawTurnsTheCameraCounterClockwiseAboutVertical() {
    CameraMount left = mount(0, 0, 0, 90.0, 0);
    // Facing left: "out" is robot-left, camera-right is robot-forward.
    assertVec("out", new double[] {0, 10, 0}, HiveVision.cameraToRobot(0, 10, 0, left));
    assertVec("right", new double[] {10, 0, 0}, HiveVision.cameraToRobot(10, 0, 0, left));
    assertVec("up", new double[] {0, 0, 10}, HiveVision.cameraToRobot(0, 0, 10, left));

    CameraMount back = mount(0, 0, 0, 180.0, 0);
    assertVec(
        "facing backward", new double[] {-10, 0, 0}, HiveVision.cameraToRobot(0, 10, 0, back));

    CameraMount diagonal = mount(0, 0, 0, 45.0, 0);
    double r = Math.sqrt(0.5) * 10;
    assertVec(
        "45 degrees left", new double[] {r, r, 0}, HiveVision.cameraToRobot(0, 10, 0, diagonal));
  }

  @Test
  public void testPitchTiltsTheLensUpAndTheImageTopBackward() {
    CameraMount up45 = mount(0, 0, 0, 0, 45.0);
    double r = Math.sqrt(0.5) * 10;
    assertVec(
        "out of the lens climbs", new double[] {r, 0, r}, HiveVision.cameraToRobot(0, 10, 0, up45));
    assertVec(
        "top of the image leans back",
        new double[] {-r, 0, r},
        HiveVision.cameraToRobot(0, 0, 10, up45));
    assertVec(
        "sideways is unaffected",
        new double[] {0, -10, 0},
        HiveVision.cameraToRobot(10, 0, 0, up45));

    CameraMount straightUp = mount(0, 0, 0, 0, 90.0);
    assertVec(
        "looking at the ceiling",
        new double[] {0, 0, 10},
        HiveVision.cameraToRobot(0, 10, 0, straightUp));
    assertVec(
        "image top points backward",
        new double[] {-10, 0, 0},
        HiveVision.cameraToRobot(0, 0, 10, straightUp));
  }

  @Test
  public void testTheShippedMountPitchRaisesADistantTargetAboveTheCamera() {
    // A target 40 in out along the lens axis of a camera pitched up 50 degrees sits well above it:
    // that is how the upward CELL (59.6 in) is seen from a camera 7.7 in off the floor.
    CameraMount m = mount(0, 0, 7.68, 0, 50.0);
    double[] p = HiveVision.cameraToRobot(0, 40, 0, m);
    assertEquals(40 * Math.cos(Math.toRadians(50)), p[0], EPS);
    assertEquals(7.68 + 40 * Math.sin(Math.toRadians(50)), p[2], EPS);
  }

  @Test
  public void testCameraAxesStayOrthonormalAndRightHandedForAnyMount() {
    for (double yaw : new double[] {-170, -90, -30, 0, 15, 90, 135, 180}) {
      for (double pitch : new double[] {-60, -10, 0, 25, 50, 80}) {
        CameraMount m = mount(0, 0, 0, yaw, pitch);
        double[] right = HiveVision.cameraToRobot(1, 0, 0, m);
        double[] out = HiveVision.cameraToRobot(0, 1, 0, m);
        double[] up = HiveVision.cameraToRobot(0, 0, 1, m);
        String at = " (yaw " + yaw + ", pitch " + pitch + ")";

        assertEquals(1.0, norm(right), EPS, "|right|" + at);
        assertEquals(1.0, norm(out), EPS, "|out|" + at);
        assertEquals(1.0, norm(up), EPS, "|up|" + at);
        assertEquals(0.0, dot(right, out), EPS, "right.out" + at);
        assertEquals(0.0, dot(right, up), EPS, "right.up" + at);
        assertEquals(0.0, dot(out, up), EPS, "out.up" + at);

        // x (right) cross y (out) must be z (up): a mirrored mapping would flip left and right.
        double[] cross = {
          right[1] * out[2] - right[2] * out[1],
          right[2] * out[0] - right[0] * out[2],
          right[0] * out[1] - right[1] * out[0]
        };
        assertVec("right x out = up" + at, up, cross);
      }
    }
  }

  @Test
  public void testMountOffsetsAddAfterTheRotation() {
    CameraMount m = mount(3.54, 4.72, 7.68, 0.0, 50.0);
    double[] origin = HiveVision.cameraToRobot(0, 0, 0, m);
    double[] p = HiveVision.cameraToRobot(2, 30, -4, m);
    CameraMount atZero = mount(0, 0, 0, 0.0, 50.0);
    double[] q = HiveVision.cameraToRobot(2, 30, -4, atZero);
    assertVec(
        "offset is a pure translation", new double[] {q[0] + 3.54, q[1] + 4.72, q[2] + 7.68}, p);
    assertVec("origin", new double[] {3.54, 4.72, 7.68}, origin);
  }

  @Test
  public void testCameraMountFromConfig() {
    CameraMount m = CameraMount.fromConfig();
    var c = config.vision.camera;
    assertEquals(c.forward_in, m.forwardIn(), 0.0);
    assertEquals(c.left_in, m.leftIn(), 0.0);
    assertEquals(c.height_in, m.heightIn(), 0.0);
    assertEquals(Math.toRadians(c.yaw_deg), m.yawRad(), 0.0);
    assertEquals(Math.toRadians(c.pitch_up_deg), m.pitchUpRad(), 0.0);
    assertTrue(m.pitchUpRad() > 0.0, "the BIOBUZZ tags are above the camera, so it must look up");
  }

  /** The inverse of cameraToRobot, written independently from the axis definitions. */
  private static double[] robotToCamera(double[] robotPoint, CameraMount m) {
    double dF = robotPoint[0] - m.forwardIn();
    double dL = robotPoint[1] - m.leftIn();
    double dU = robotPoint[2] - m.heightIn();
    double cy = Math.cos(m.yawRad());
    double sy = Math.sin(m.yawRad());
    double cp = Math.cos(m.pitchUpRad());
    double sp = Math.sin(m.pitchUpRad());
    // Camera axes in the robot frame.
    double[] right = {sy, -cy, 0.0};
    double[] out = {cp * cy, cp * sy, sp};
    double[] up = {-cy * sp, -sy * sp, cp};
    double[] d = {dF, dL, dU};
    return new double[] {dot(d, right), dot(d, out), dot(d, up)};
  }

  @Test
  public void testRobotPointRoundTripsThroughTheCameraFrame() {
    double[][] points = {{40, 0, 59.56}, {30, -12, 38.17}, {55, 20, 59.56}, {18, 5, 45}};
    for (double yaw : new double[] {-35, 0, 20}) {
      for (double pitch : new double[] {0, 30, 50}) {
        CameraMount m = mount(3.54, 4.72, 7.68, yaw, pitch);
        for (double[] p : points) {
          double[] cam = robotToCamera(p, m);
          assertVec(
              "yaw " + yaw + " pitch " + pitch,
              p,
              HiveVision.cameraToRobot(cam[0], cam[1], cam[2], m));
        }
      }
    }
  }

  // -------------------------------------------------------------------------------------------
  // isUpward

  @Test
  public void testIsUpwardSplitsAtTheThresholdInclusively() {
    double threshold = config.hive.up_height_threshold_in;

    assertTrue(
        HiveVision.isUpward(sighting(40, 0, config.field.hive.up_opening_height_in), threshold));
    assertFalse(
        HiveVision.isUpward(sighting(40, 0, config.field.hive.down_opening_height_in), threshold));
    assertTrue(
        HiveVision.isUpward(sighting(40, 0, threshold), threshold),
        "exactly on the line counts as up");
    assertFalse(HiveVision.isUpward(sighting(40, 0, threshold - 1e-6), threshold));
    assertTrue(HiveVision.isUpward(sighting(40, 0, threshold + 1e-6), threshold));
  }

  @Test
  public void testSightingDerivedRangeAndBearing() {
    CellSighting s = sighting(30, 40, 59.56);
    assertEquals(50.0, s.rangeIn(), EPS);
    assertEquals(Math.atan2(40, 30), s.bearingRad(), EPS);
    assertEquals(0.0, sighting(25, 0, 1).bearingRad(), EPS);
    assertEquals(Math.PI / 2, sighting(0, 12, 1).bearingRad(), EPS);
    assertEquals(
        -Math.PI / 2,
        sighting(0, -12, 1).bearingRad(),
        EPS,
        "a CELL to the robot's right has negative bearing");
  }

  /** A full chain: place a CELL in the world, look at it through the camera, classify it. */
  @Test
  public void testUpwardAndDownwardCellsAreClassifiedThroughTheCamera() {
    CameraMount m = CameraMount.fromConfig();
    FieldModel field = new FieldModel(Alliance.RED);
    Pose robot = new Pose(58.0, 27.0, Math.PI / 2);
    double threshold = config.hive.up_height_threshold_in;

    for (boolean upward : new boolean[] {true, false}) {
      FieldModel.Opening o = field.opening(HiveCell.A, upward);
      CellSighting s = sightingOf(robot, o, m);
      assertEquals(o.heightInches(), s.heightIn(), 1e-6, "height survives the camera round trip");
      assertEquals(upward, HiveVision.isUpward(s, threshold));
    }
  }

  // -------------------------------------------------------------------------------------------
  // relocalize

  /** Builds the sighting a camera would report for an opening seen from {@code robot}. */
  private static CellSighting sightingOf(Pose robot, FieldModel.Opening opening, CameraMount m) {
    double dx = opening.x() - robot.x();
    double dy = opening.y() - robot.y();
    double c = Math.cos(robot.heading());
    double s = Math.sin(robot.heading());
    // Rotate the field-frame offset into the robot frame.
    double forward = dx * c + dy * s;
    double left = -dx * s + dy * c;
    double[] cam = robotToCamera(new double[] {forward, left, opening.heightInches()}, m);
    double[] back = HiveVision.cameraToRobot(cam[0], cam[1], cam[2], m);
    return new CellSighting(Alliance.RED, HiveCell.A, back[0], back[1], back[2], 123L, 90);
  }

  @Test
  public void testRelocalizeRecoversAKnownPoseFromEveryCellOfBothAlliances() {
    CameraMount m = CameraMount.fromConfig();
    Pose[] robots = {
      new Pose(58.0, 27.0, Math.PI / 2),
      new Pose(40.0, 30.0, Math.toRadians(70)),
      new Pose(75.0, 110.0, Math.toRadians(-100)),
      new Pose(36.0, 106.0, Math.toRadians(318.36)),
      new Pose(120.0, 20.0, Math.toRadians(150))
    };
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      for (HiveCell cell : HiveCell.values()) {
        FieldModel.Opening opening = field.upOpening(cell);
        for (Pose robot : robots) {
          CellSighting s = sightingOf(robot, opening, m);
          Pose back = HiveVision.relocalize(s, opening, robot.heading());
          String what = alliance + " " + cell + " from " + robot;
          assertEquals(robot.x(), back.x(), 1e-6, what);
          assertEquals(robot.y(), back.y(), 1e-6, what);
          assertEquals(robot.heading(), back.heading(), 0.0, what);
        }
      }
    }
  }

  @Test
  public void testRelocalizePlacesTheRobotBehindTheOpeningAlongItsHeading() {
    FieldModel.Opening opening = new FieldModel(Alliance.RED).upOpening(HiveCell.A);
    // 30 in dead ahead of a robot facing +y means the robot is 30 in below the opening.
    Pose p = HiveVision.relocalize(sighting(30, 0, 59.56), opening, Math.PI / 2);
    assertEquals(opening.x(), p.x(), 1e-9);
    assertEquals(opening.y() - 30.0, p.y(), 1e-9);
    assertEquals(Math.PI / 2, p.heading(), 0.0);

    // The same sighting while facing +x puts the robot 30 in to the opening's -x side.
    Pose q = HiveVision.relocalize(sighting(30, 0, 59.56), opening, 0.0);
    assertEquals(opening.x() - 30.0, q.x(), 1e-9);
    assertEquals(opening.y(), q.y(), 1e-9);

    // An opening 10 in to the robot's left, facing +y: the robot is 10 in to its right (+x).
    Pose r = HiveVision.relocalize(sighting(0, 10, 59.56), opening, Math.PI / 2);
    assertEquals(opening.x() + 10.0, r.x(), 1e-9);
    assertEquals(opening.y(), r.y(), 1e-9);
  }

  @Test
  public void testRelocalizeUsesTheOdometryHeadingNotTheSightingBearing() {
    FieldModel.Opening opening = new FieldModel(Alliance.BLUE).upOpening(HiveCell.B);
    double heading = AngleUnit.normalizeRadians(Math.toRadians(215));
    Pose p = HiveVision.relocalize(sighting(25, -8, 59.56), opening, heading);
    assertEquals(0.0, AngleUnit.normalizeRadians(heading - p.heading()), EPS);
    // Distance from the robot to the opening is the sighting's range, wherever the robot is.
    assertEquals(Math.hypot(25, -8), Math.hypot(opening.x() - p.x(), opening.y() - p.y()), 1e-9);
  }

  @Test
  public void testRelocalizationFrameFreshness() {
    CellSighting frame = new CellSighting(Alliance.RED, HiveCell.A, 30, 0, 59, 1_000_000_000L, 100);
    assertTrue(HiveVision.isFresh(frame, 1000, 400));
    assertTrue(HiveVision.isFresh(frame, 1400, 400));
    assertFalse(HiveVision.isFresh(frame, 1401, 400));
    assertFalse(HiveVision.isFresh(frame, 999, 400));
  }
}
