package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ServoMotorSubsystem;

/** Initializes and tests only the continuous-rotation servo motor. */
@TeleOp(name = "Test: Servo Motor", group = "Diagnostics")
public final class ServoMotorTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    ServoMotorSubsystem servoMotor = new ServoMotorSubsystem(hardwareMap);

    telemetry.addLine("Press B on either gamepad to toggle the servo motor.");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    waitForStart();

    boolean previousB = false;
    try {
      while (opModeIsActive()) {
        boolean bDown = gamepad1.b || gamepad2.b;
        if (bDown && !previousB) {
          servoMotor.toggleEnabled();
        }
        previousB = bDown;

        telemetry.addData("Servo Motor", servoMotor.isEnabled() ? "ON" : "OFF");
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      servoMotor.stop();
    }
  }
}
