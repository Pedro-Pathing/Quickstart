package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Reads only the hub IMU. */
@TeleOp(name = "Test: IMU", group = "Diagnostics")
public final class ImuTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    IMU imu = hardwareMap.get(IMU.class, RobotConfig.Hardware.IMU);
    imu.initialize(
        new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

    telemetry.addLine("Press A on either gamepad to reset yaw.");
    telemetry.update();
    waitForStart();

    boolean previousA = false;
    while (opModeIsActive()) {
      boolean aDown = gamepad1.a || gamepad2.a;
      if (aDown && !previousA) {
        imu.resetYaw();
      }
      previousA = aDown;

      YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
      telemetry.addData("Yaw", "%.1f°", angles.getYaw(AngleUnit.DEGREES));
      telemetry.addData("Pitch", "%.1f°", angles.getPitch(AngleUnit.DEGREES));
      telemetry.addData("Roll", "%.1f°", angles.getRoll(AngleUnit.DEGREES));
      telemetry.update();
      idle();
    }
  }
}
