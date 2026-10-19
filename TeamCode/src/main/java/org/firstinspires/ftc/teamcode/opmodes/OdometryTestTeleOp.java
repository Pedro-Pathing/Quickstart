package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.localization.PinpointOdometryLocalizer;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;

/** Tests the Pinpoint and its two pods without initializing drive motors or the hub IMU. */
@TeleOp(name = "Test: Odometry Wheels", group = "Diagnostics")
public final class OdometryTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    telemetry.addLine("Keep the robot still while Pinpoint calibrates.");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    PinpointOdometryLocalizer odometry = new PinpointOdometryLocalizer(hardwareMap);
    GoBildaPinpointDriver pinpoint = odometry.getPinpoint();
    odometry.update();
    int forwardZero = pinpoint.getEncoderX();
    int strafeZero = pinpoint.getEncoderY();

    telemetry.addLine("Push the robot by hand or rotate each pod separately.");
    telemetry.addLine("A: zero displayed pose and pod counts (no gyro recalibration).");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    waitForStart();

    boolean previousA = false;
    while (opModeIsActive()) {
      odometry.update();
      boolean aDown = gamepad1.a || gamepad2.a;
      if (aDown && !previousA) {
        odometry.resetPose(0, 0, 0);
        forwardZero = pinpoint.getEncoderX();
        strafeZero = pinpoint.getEncoderY();
      }
      previousA = aDown;

      int forwardTicks = pinpoint.getEncoderX() - forwardZero;
      int strafeTicks = pinpoint.getEncoderY() - strafeZero;
      PoseEstimate pose = odometry.getPoseEstimate();
      telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
      telemetry.addData("Connection", pinpoint.getConnectionInfo());
      telemetry.addData("Required Bus", "Control Hub I2C %d", RobotConfig.Hardware.PINPOINT_I2C_BUS);
      telemetry.addData("Forward Pod Ticks", forwardTicks);
      telemetry.addData("Forward Pod Distance", "%.2f in", forwardTicks / RobotConfig.Pinpoint.TICKS_PER_INCH);
      telemetry.addData("Strafe Pod Ticks", strafeTicks);
      telemetry.addData("Strafe Pod Distance", "%.2f in", strafeTicks / RobotConfig.Pinpoint.TICKS_PER_INCH);
      telemetry.addData("Pinpoint Pose", "X %.2f, Y %.2f in; heading %.1f deg",
          pose.getX(), pose.getY(), pose.getHeadingDegrees());
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      idle();
    }
  }
}
