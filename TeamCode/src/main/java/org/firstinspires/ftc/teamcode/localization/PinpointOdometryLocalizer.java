package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Pinpoint pose adapter for the legacy OpModes and isolated odometry diagnostic. */
public final class PinpointOdometryLocalizer {
  private final PinpointLocalizer localizer;

  public PinpointOdometryLocalizer(HardwareMap hardwareMap) {
    localizer = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants());
    localizer.resetIMU();
    resetPose(0, 0, 0);
  }

  public void update() { localizer.update(); }

  /** Changes the pose origin without recalibrating the Pinpoint gyro. */
  public void resetPose(double x, double y, double headingDegrees) {
    localizer.setPose(new Pose(x, y, Math.toRadians(headingDegrees)));
  }

  public PoseEstimate getPoseEstimate() {
    Pose pose = localizer.getPose();
    return new PoseEstimate(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
  }

  public GoBildaPinpointDriver getPinpoint() { return localizer.getPinpoint(); }
}
