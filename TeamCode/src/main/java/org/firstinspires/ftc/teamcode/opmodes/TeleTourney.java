package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroAprilTagValidation;
import org.firstinspires.ftc.teamcode.telemetry.PedroValidationTelemetry;

/** Competition drive TeleOp with live two-wheel odometry and Limelight AprilTag telemetry. */
@TeleOp(name = "TeleTourney", group = "Competition")
public final class TeleTourney extends LinearOpMode {
  @Override
  public void runOpMode() {
    Follower follower = Constants.createFollower(hardwareMap);
    DcMotor forwardEncoder = hardwareMap.get(DcMotor.class, RobotConfig.Hardware.ODOMETRY_FORWARD);
    DcMotor strafeEncoder = hardwareMap.get(DcMotor.class, RobotConfig.Hardware.ODOMETRY_STRAFE);
    AprilTagTracker aprilTags = new AprilTagTracker(hardwareMap);
    PedroAprilTagValidation validation = new PedroAprilTagValidation(
        RobotConfig.Localization.VALIDATION_POSITION_TOLERANCE_INCHES,
        RobotConfig.Localization.VALIDATION_HEIGHT_TOLERANCE_INCHES, true);

    telemetry.setMsTransmissionInterval(50);
    telemetry.addData("Status", "Drive, odometry, and Limelight initialized");
    telemetry.addLine("Left stick: drive/strafe | Triggers: rotate");
    telemetry.addLine("First visible tag anchors the check. A: re-anchor | B: clear");
    telemetry.update();
    waitForStart();

    try {
      if (isStopRequested()) {
        return;
      }
      follower.setStartingPose(new Pose(0, 0, 0));
      follower.setPose(new Pose(0, 0, 0));
      follower.startTeleopDrive(true);
      aprilTags.start();
      while (opModeIsActive()) {
        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        // Pedro uses positive left strafe and counterclockwise turn; the mapper uses right/CW.
        follower.setTeleOpDrive(command.getDrive(), -command.getStrafe(), -command.getRotate(), true);

        follower.update();
        aprilTags.update();
        Pose pose = follower.getPose();
        List<AprilTagObservation> visibleTags = aprilTags.getObservations();
        AprilTagObservation nearestTag = aprilTags.getNearestObservation();
        validation.update(pose, visibleTags, aprilTags.isConnected(),
            gamepad1.a || gamepad2.a, gamepad1.b || gamepad2.b);

        telemetry.addData("Active Driver", "Gamepad %d", command.getGamepadNumber());
        telemetry.addData(
            "Drive / Strafe / Rotate",
            "%.2f / %.2f / %.2f",
            command.getDrive(),
            command.getStrafe(),
            command.getRotate());
        telemetry.addData("Field Position", "X %.2f in, Y %.2f in", pose.getX(), pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Forward Odo", "%d ticks", forwardEncoder.getCurrentPosition());
        telemetry.addData("Strafe Odo", "%d ticks", strafeEncoder.getCurrentPosition());
        telemetry.addData("Pose Source", "Pedro: 2-wheel odometry + IMU");
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
        PedroValidationTelemetry.add(telemetry, validation);
        telemetry.update();
        idle();
      }
    } finally {
      follower.breakFollowing();
      aprilTags.stop();
    }
  }
}
