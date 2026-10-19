package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;

/** Initializes and tests only Limelight AprilTag detection. */
@TeleOp(name = "Test: Limelight", group = "Diagnostics")
public final class LimelightTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    AprilTagTracker tracker = new AprilTagTracker(hardwareMap);
    List<AprilTagObservation> displayedTags = Collections.emptyList();
    boolean frozen = false;
    boolean previousA = false;

    tracker.start();
    try {
      telemetry.addLine("Press A on either gamepad to freeze/unfreeze measurements.");
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();

      while (opModeIsActive()) {
        tracker.update();
        boolean aDown = gamepad1.a || gamepad2.a;
        if (aDown && !previousA) {
          frozen = !frozen;
        }
        previousA = aDown;
        if (!frozen) {
          displayedTags = tracker.getObservations();
        }

        telemetry.addData("Connected", tracker.isConnected());
        telemetry.addData("Display", frozen ? "FROZEN" : "LIVE");
        telemetry.addData("Tags", displayedTags.size());
        for (AprilTagObservation tag : displayedTags) {
          telemetry.addData(
              "Tag " + tag.getId(),
              "forward %.1f, left %.1f, range %.1f, height %.1f in",
              tag.getForwardInches(),
              tag.getLeftInches(),
              tag.getRangeInches(),
              tag.getHeightInches());
        }
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      tracker.stop();
    }
  }
}
