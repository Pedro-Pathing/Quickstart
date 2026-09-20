package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Reads only the two odometry pod encoders. */
@TeleOp(name = "Test: Odometry Wheels", group = "Diagnostics")
public final class OdometryTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    DcMotor forwardEncoder =
        hardwareMap.get(DcMotor.class, RobotConfig.Hardware.ODOMETRY_FORWARD);
    DcMotor strafeEncoder =
        hardwareMap.get(DcMotor.class, RobotConfig.Hardware.ODOMETRY_STRAFE);
    double forwardZero = adjustedForwardTicks(forwardEncoder);
    double strafeZero = adjustedStrafeTicks(strafeEncoder);

    telemetry.addLine("Push the robot by hand or rotate each pod separately.");
    telemetry.addLine("Press A on either gamepad to zero the readings.");
    telemetry.update();
    waitForStart();

    boolean previousA = false;
    while (opModeIsActive()) {
      boolean aDown = gamepad1.a || gamepad2.a;
      if (aDown && !previousA) {
        forwardZero = adjustedForwardTicks(forwardEncoder);
        strafeZero = adjustedStrafeTicks(strafeEncoder);
      }
      previousA = aDown;

      double forwardTicks = adjustedForwardTicks(forwardEncoder) - forwardZero;
      double strafeTicks = adjustedStrafeTicks(strafeEncoder) - strafeZero;

      telemetry.addData("Forward Pod Ticks", "%.0f", forwardTicks);
      telemetry.addData(
          "Forward Pod Distance",
          "%.2f in",
          forwardTicks / RobotConfig.Localization.ODOMETRY_FORWARD_TICKS_PER_INCH);
      telemetry.addData("Strafe Pod Ticks", "%.0f", strafeTicks);
      telemetry.addData(
          "Strafe Pod Distance",
          "%.2f in",
          strafeTicks / RobotConfig.Localization.ODOMETRY_STRAFE_TICKS_PER_INCH);
      telemetry.update();
      idle();
    }
  }

  private static double adjustedForwardTicks(DcMotor encoder) {
    return encoder.getCurrentPosition()
        * RobotConfig.Localization.ODOMETRY_FORWARD_MULTIPLIER;
  }

  private static double adjustedStrafeTicks(DcMotor encoder) {
    return encoder.getCurrentPosition()
        * RobotConfig.Localization.ODOMETRY_STRAFE_MULTIPLIER;
  }
}
