package org.firstinspires.ftc.teamcode.utilities;

import android.util.Size;
import androidx.annotation.Nullable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * BIOBUZZ AprilTag pipeline: finds the HIVE CELL clusters and reports where each CELL's opening is
 * relative to the robot.
 *
 * <p>The SDK's BIOBUZZ library defines each four-tag cluster with its origin at the CELL opening
 * centre and with no field position, because the CELLS move with every TIP (SDK 12 release notes).
 * So, unlike DECODE, a detection is not a field pose: it is "the opening is here, relative to the
 * camera". This class turns that into robot-frame coordinates and height, which answers two
 * questions:
 *
 * <ul>
 *   <li>Is that CELL upward-facing? Upward openings sit ~59.6 in above the TILES and downward ones
 *       ~38.2 in, so height alone classifies them ({@link #isUpward}).
 *   <li>Where is the robot? Only when the HIVE is at rest is a CELL opening a fixed landmark; then
 *       the opening's known field position and the odometry heading give the robot's position
 *       ({@link #relocalize}).
 * </ul>
 */
public final class HiveVision {

  /** Camera extrinsics in the robot frame: inches, radians, yaw CCW from forward, pitch up. */
  public record CameraMount(
      double forwardIn, double leftIn, double heightIn, double yawRad, double pitchUpRad) {

    public static CameraMount fromConfig() {
      var c = config.vision.camera;
      return new CameraMount(
          c.forward_in,
          c.left_in,
          c.height_in,
          Math.toRadians(c.yaw_deg),
          Math.toRadians(c.pitch_up_deg));
    }
  }

  /**
   * One CELL opening seen by the camera, in the robot frame (inches above the TILES for height).
   */
  public record CellSighting(
      Alliance owner,
      HiveCell cell,
      double forwardIn,
      double leftIn,
      double heightIn,
      long capturedAtNanos,
      int percentFound) {

    /** Horizontal distance from the robot centre to the opening centre. */
    public double rangeIn() {
      return Math.hypot(forwardIn, leftIn);
    }

    /** Bearing of the opening from robot forward, counter-clockwise positive. */
    public double bearingRad() {
      return Math.atan2(leftIn, forwardIn);
    }
  }

  private final CameraMount mount;
  private @Nullable AprilTagProcessor aprilTag;
  private @Nullable VisionPortal visionPortal;
  private boolean streaming = false;

  public HiveVision() {
    this(CameraMount.fromConfig());
  }

  public HiveVision(CameraMount mount) {
    this.mount = mount;
  }

  public CameraMount mount() {
    return mount;
  }

  /** Opens the camera. A missing webcam leaves vision disabled rather than failing init. */
  public void init(HardwareMap hardwareMap) {
    if (!config.vision.enabled) {
      return;
    }
    WebcamName webcam = hardwareMap.tryGet(WebcamName.class, config.vision.camera_name);
    if (webcam == null) {
      return;
    }
    aprilTag =
        new AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .build();
    aprilTag.setDecimation((float) config.vision.decimation);

    visionPortal =
        new VisionPortal.Builder()
            .setCamera(webcam)
            .setCameraResolution(new Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build();
    streaming = true;
    if (!config.vision.continuous) {
      stopStreaming();
    }
  }

  public boolean isAvailable() {
    return aprilTag != null;
  }

  /** Every BIOBUZZ CELL cluster in the latest frame, either alliance. */
  public List<CellSighting> poll() {
    List<CellSighting> sightings = new ArrayList<>();
    if (aprilTag == null || !streaming) {
      return sightings;
    }
    List<AprilTagDetection> detections = aprilTag.getDetections();
    if (detections == null) {
      return sightings;
    }
    for (AprilTagDetection detection : detections) {
      if (!(detection instanceof AprilTagClusterDetection cluster)
          || cluster.metadata == null
          || cluster.ftcPose == null) {
        continue;
      }
      Optional<FieldModel.TagOwner> owner = FieldModel.clusterOwner(cluster.metadata.name);
      if (owner.isEmpty()) {
        continue;
      }
      double[] p = cameraToRobot(cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z, mount);
      sightings.add(
          new CellSighting(
              owner.get().alliance(),
              owner.get().cell(),
              p[0],
              p[1],
              p[2],
              cluster.frameAcquisitionNanoTime,
              cluster.percentClusterFound));
    }
    return sightings;
  }

  public void resumeStreaming() {
    if (visionPortal != null && !streaming) {
      try {
        visionPortal.resumeStreaming();
        streaming = true;
      } catch (RuntimeException ignored) {
        // Camera not ready yet; the next request retries.
      }
    }
  }

  public void stopStreaming() {
    if (visionPortal != null && streaming) {
      try {
        visionPortal.stopStreaming();
        streaming = false;
      } catch (RuntimeException ignored) {
        // Camera not ready yet; the next request retries.
      }
    }
  }

  /** Releases the camera. Called from Robot.shutdown(); safe to call twice. */
  public void close() {
    if (visionPortal != null) {
      try {
        visionPortal.close();
      } catch (RuntimeException ignored) {
        // Already closed.
      }
      visionPortal = null;
      aprilTag = null;
      streaming = false;
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Pure geometry

  /**
   * Converts an SDK {@code ftcPose} position (x right, y out of the lens, z up, relative to the
   * camera) into the robot frame.
   *
   * @return {forward, left, height above the TILES}, inches
   */
  public static double[] cameraToRobot(double camX, double camY, double camZ, CameraMount m) {
    double cy = Math.cos(m.yawRad());
    double sy = Math.sin(m.yawRad());
    double cp = Math.cos(m.pitchUpRad());
    double sp = Math.sin(m.pitchUpRad());

    // Camera axes expressed in the robot frame (forward, left, up).
    double[] right = {sy, -cy, 0.0};
    double[] out = {cp * cy, cp * sy, sp};
    double[] up = {-cy * sp, -sy * sp, cp};

    return new double[] {
      m.forwardIn() + camX * right[0] + camY * out[0] + camZ * up[0],
      m.leftIn() + camX * right[1] + camY * out[1] + camZ * up[1],
      m.heightIn() + camX * right[2] + camY * out[2] + camZ * up[2]
    };
  }

  /** Rejects old or future-dated camera frames before a relocalization. */
  public static boolean isFresh(CellSighting sighting, long nowMs, long maxAgeMs) {
    long ageMs = nowMs - sighting.capturedAtNanos() / 1_000_000L;
    return ageMs >= 0 && ageMs <= maxAgeMs;
  }

  /** Whether a sighted CELL opening is high enough to be the upward-facing CELL. */
  public static boolean isUpward(CellSighting sighting, double thresholdIn) {
    return sighting.heightIn() >= thresholdIn;
  }

  /**
   * Robot pose implied by seeing a CELL opening whose field position is known (the HIVE is at rest
   * in a known state). Heading comes from odometry: the Pinpoint IMU heading is far better than a
   * single cluster's yaw, and the opening's field position pins x and y.
   */
  public static Pose relocalize(
      CellSighting sighting, FieldModel.Opening opening, double robotHeadingRad) {
    double c = Math.cos(robotHeadingRad);
    double s = Math.sin(robotHeadingRad);
    double dx = sighting.forwardIn() * c - sighting.leftIn() * s;
    double dy = sighting.forwardIn() * s + sighting.leftIn() * c;
    return new Pose(opening.x() - dx, opening.y() - dy, robotHeadingRad);
  }
}
