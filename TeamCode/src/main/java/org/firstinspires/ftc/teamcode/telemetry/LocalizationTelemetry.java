package org.firstinspires.ftc.teamcode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;
import org.firstinspires.ftc.teamcode.localization.RobotLocalization;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;

/** Formats localization state for Driver Station telemetry and FTC Dashboard. */
public final class LocalizationTelemetry {
  private static final double ROBOT_RADIUS_INCHES = 7.0;
  private final FtcDashboard dashboard;

  public LocalizationTelemetry() {
    dashboard = FtcDashboard.getInstance();
  }

  public void add(Telemetry telemetry, RobotLocalization localization) {
    add(telemetry, localization, null);
  }

  public void add(Telemetry telemetry, RobotLocalization localization, Waypoint target) {
    PoseEstimate pose = localization.getPoseEstimate();
    List<AprilTagObservation> aprilTags = localization.getAprilTagObservations();

    telemetry.addData("Pose Source", "Odometry");
    telemetry.addData("Field X (in)", "%.2f", pose.getX());
    telemetry.addData("Field Y (in)", "%.2f", pose.getY());
    telemetry.addData("Heading (deg)", "%.1f", pose.getHeadingDegrees());
    telemetry.addData("LL Connected", localization.isLimelightConnected());
    telemetry.addData("Tags Detected", aprilTags.size());
    for (AprilTagObservation tag : aprilTags) {
      telemetry.addData(
          "Tag " + tag.getId(),
          "range %.1f in, horizontal %.1f in, height %.1f in",
          tag.getRangeInches(),
          tag.getHorizontalDistanceInches(),
          tag.getHeightInches());
    }

    sendDashboard(pose, aprilTags, localization.getNearestAprilTag(), target);
  }

  private void sendDashboard(
      PoseEstimate pose,
      List<AprilTagObservation> aprilTags,
      AprilTagObservation nearestTag,
      Waypoint target) {
    TelemetryPacket packet = new TelemetryPacket();
    OdometryTelemetry.add(packet);
    packet.put("x", pose.getX());
    packet.put("y", pose.getY());
    packet.put("heading (deg)", pose.getHeadingDegrees());
    packet.put("poseSource", "ODOMETRY");
    packet.put("tagCount", aprilTags.size());

    if (nearestTag != null) {
      packet.put("nearestTagId", nearestTag.getId());
      packet.put("nearestTagRange (in)", nearestTag.getRangeInches());
      packet.put("nearestTagHeight (in)", nearestTag.getHeightInches());
    }

    Canvas fieldOverlay = packet.fieldOverlay();
    drawRobot(fieldOverlay, pose);
    if (target != null) {
      drawTarget(fieldOverlay, pose, target);
    }

    dashboard.sendTelemetryPacket(packet);
  }

  private static void drawRobot(Canvas fieldOverlay, PoseEstimate pose) {
    double headingRadians = Math.toRadians(pose.getHeadingDegrees());

    fieldOverlay.setStrokeWidth(2);
    fieldOverlay.setStroke("#2196F3");
    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS_INCHES);

    double frontX = pose.getX() + ROBOT_RADIUS_INCHES * Math.cos(headingRadians);
    double frontY = pose.getY() + ROBOT_RADIUS_INCHES * Math.sin(headingRadians);
    fieldOverlay.setStroke("#FFFFFF");
    fieldOverlay.setStrokeWidth(3);
    fieldOverlay.strokeLine(pose.getX(), pose.getY(), frontX, frontY);
  }

  private static void drawTarget(Canvas fieldOverlay, PoseEstimate pose, Waypoint target) {
    fieldOverlay.setStroke("#F44336");
    fieldOverlay.setStrokeWidth(2);
    fieldOverlay.strokeCircle(target.getX(), target.getY(), 3.0);
    fieldOverlay.strokeLine(target.getX() - 4, target.getY(), target.getX() + 4, target.getY());
    fieldOverlay.strokeLine(target.getX(), target.getY() - 4, target.getX(), target.getY() + 4);

    fieldOverlay.setStroke("#F4433666");
    fieldOverlay.setStrokeWidth(1);
    fieldOverlay.strokeLine(pose.getX(), pose.getY(), target.getX(), target.getY());
  }
}
