package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;
import org.firstinspires.ftc.teamcode.localization.TwoWheelOdometryLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/** Competition drive TeleOp with live two-wheel odometry and Limelight AprilTag telemetry. */
@TeleOp(name = "TeleTourney", group = "Competition")
public final class TeleTourney extends LinearOpMode {
  @Override
  public void runOpMode() {
    MecanumDrive drive = new MecanumDrive(hardwareMap);
    TwoWheelOdometryLocalizer odometry = new TwoWheelOdometryLocalizer(hardwareMap);
    AprilTagTracker aprilTags = new AprilTagTracker(hardwareMap);

    telemetry.setMsTransmissionInterval(50);
    telemetry.addData("Status", "Drive, odometry, and Limelight initialized");
    telemetry.addLine("Left stick: drive/strafe | Triggers: rotate");
    telemetry.update();
    waitForStart();

    odometry.resetPose(0, 0, 0);
    aprilTags.start();
    try {
      while (opModeIsActive()) {
        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        drive.driveRobotCentric(command.getDrive(), command.getStrafe(), command.getRotate());

        odometry.update();
        aprilTags.update();
        PoseEstimate pose = odometry.getPoseEstimate();
        List<AprilTagObservation> visibleTags = aprilTags.getObservations();
        AprilTagObservation nearestTag = aprilTags.getNearestObservation();

        telemetry.addData("Active Driver", "Gamepad %d", command.getGamepadNumber());
        telemetry.addData(
            "Drive / Strafe / Rotate",
            "%.2f / %.2f / %.2f",
            command.getDrive(),
            command.getStrafe(),
            command.getRotate());
        telemetry.addData("Field Position", "X %.2f in, Y %.2f in", pose.getX(), pose.getY());
        telemetry.addData("Heading", "%.1f°", pose.getHeadingDegrees());
        telemetry.addData("Forward Odo", "%.0f ticks", odometry.getForwardEncoderTicks());
        telemetry.addData("Strafe Odo", "%.0f ticks", odometry.getStrafeEncoderTicks());
        telemetry.addData("Pose Source", "2-wheel odometry + IMU");
        telemetry.addData("Limelight", aprilTags.isConnected() ? "CONNECTED" : "DISCONNECTED");
        telemetry.addData("AprilTags", visibleTags.size());
        if (nearestTag == null) {
          telemetry.addData("Nearest Tag", "None visible");
        } else {
          telemetry.addData("Nearest Tag", "ID %d", nearestTag.getId());
          telemetry.addData(
              "Tag Position",
              "forward %.1f in, left %.1f in",
              nearestTag.getForwardInches(),
              nearestTag.getLeftInches());
          telemetry.addData(
              "Tag Distance",
              "horizontal %.1f in, range %.1f in",
              nearestTag.getHorizontalDistanceInches(),
              nearestTag.getRangeInches());
          telemetry.addData("Tag Height", "%.1f in", nearestTag.getHeightInches());
        }
        telemetry.update();
        idle();
      }
    } finally {
      drive.stop();
      aprilTags.stop();
    }
  }
}
