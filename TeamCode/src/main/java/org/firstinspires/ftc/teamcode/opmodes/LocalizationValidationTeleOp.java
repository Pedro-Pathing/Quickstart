package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroAprilTagValidation;
import org.firstinspires.ftc.teamcode.telemetry.PedroValidationTelemetry;

/** On-robot comparison of Pedro's Pinpoint pose and Limelight tag measurements. */
@TeleOp(name = "Test: Odo vs AprilTag", group = "Diagnostics")
public final class LocalizationValidationTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    Follower follower = Constants.createFollower(hardwareMap);
    AprilTagTracker tags = new AprilTagTracker(hardwareMap);
    PedroAprilTagValidation validation = new PedroAprilTagValidation(
        RobotConfig.Localization.VALIDATION_POSITION_TOLERANCE_INCHES,
        RobotConfig.Localization.VALIDATION_HEIGHT_TOLERANCE_INCHES, false);

    telemetry.setMsTransmissionInterval(50);
    try {
      telemetry.addLine("Pedro + Limelight: A captures a tag baseline; B clears it.");
      telemetry.addLine("Drive slowly and keep the baseline tag visible. No mechanisms operate.");
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      if (isStopRequested()) return;
      follower.setStartingPose(new Pose(0, 0, 0));
      follower.setPose(new Pose(0, 0, 0));
      follower.setMaxPower(0.35);
      follower.startTeleopDrive(true);
      tags.start();

      while (opModeIsActive()) {
        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        follower.setTeleOpDrive(command.getDrive(), -command.getStrafe(), -command.getRotate(), true);
        follower.update();
        tags.update();
        Pose pose = follower.getPose();
        validation.update(pose, tags.getObservations(), tags.isConnected(),
            gamepad1.a || gamepad2.a, gamepad1.b || gamepad2.b);

        telemetry.addData("Pose Source", "Pedro: Pinpoint (2 pods + onboard IMU)");
        telemetry.addData("Pedro Position", "X %.2f, Y %.2f in", pose.getX(), pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Active Driver", "Gamepad %d", command.getGamepadNumber());
        telemetry.addData("Limelight Connected", tags.isConnected());
        telemetry.addData("Visible Tags", tags.getObservations().size());
        PedroValidationTelemetry.add(telemetry, validation);
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      follower.breakFollowing();
      tags.stop();
    }
  }
}
