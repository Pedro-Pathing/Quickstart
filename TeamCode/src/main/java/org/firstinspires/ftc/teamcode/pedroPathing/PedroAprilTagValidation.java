package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import java.util.List;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagOdometryValidator;
import org.firstinspires.ftc.teamcode.localization.AprilTagOdometryValidator.ValidationResult;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;

/** Cross-checks Pedro translation against a fixed tag baseline, without writing follower pose. */
public final class PedroAprilTagValidation {
  private final AprilTagOdometryValidator validator;
  private boolean autoCapture;
  private boolean previousA;
  private boolean previousB;
  private boolean connected;
  private PoseEstimate visionPose;
  private ValidationResult result;

  public PedroAprilTagValidation(double positionTolerance, double heightTolerance, boolean autoCapture) {
    validator = new AprilTagOdometryValidator(positionTolerance, heightTolerance);
    this.autoCapture = autoCapture;
  }

  public void update(Pose pedroPose, List<AprilTagObservation> tags, boolean connected,
      boolean aDown, boolean bDown) {
    this.connected = connected;
    PoseEstimate pose = new PoseEstimate(pedroPose.getX(), pedroPose.getY(),
        Math.toDegrees(pedroPose.getHeading()));
    boolean capture = aDown && !previousA;
    boolean clear = bDown && !previousB;
    previousA = aDown;
    previousB = bDown;

    if (clear) {
      validator.clearBaseline();
      autoCapture = false;
    } else if (connected && (capture || (autoCapture && !validator.hasBaseline()))) {
      AprilTagObservation nearest = null;
      for (AprilTagObservation tag : tags) {
        if (nearest == null || tag.getRangeInches() < nearest.getRangeInches()) {
          nearest = tag;
        }
      }
      if (nearest != null) {
        validator.captureBaseline(pose, nearest);
        autoCapture = false;
      }
    }

    AprilTagObservation tracked = null;
    if (connected && validator.hasBaseline()) {
      for (AprilTagObservation tag : tags) {
        if (tag.getId() == validator.getTrackedTagId()) {
          tracked = tag;
          break;
        }
      }
    }
    result = validator.validate(pose, tracked);
    visionPose = validator.estimateRobotPose(pose, tracked);
  }

  public String getStatus() {
    if (!connected) return "DISCONNECTED";
    if (!validator.hasBaseline()) return "WAITING - press A with a tag visible";
    if (result == null || !result.isAvailable()) return "NO DATA - baseline tag not visible";
    return result.isPassing() ? "PASS - Pedro and Limelight agree" : "FAIL - measurements disagree";
  }

  public Integer getTrackedTagId() {
    return validator.hasBaseline() ? validator.getTrackedTagId() : null;
  }

  public ValidationResult getResult() { return result; }

  public PoseEstimate getVisionPose() { return visionPose; }
}
