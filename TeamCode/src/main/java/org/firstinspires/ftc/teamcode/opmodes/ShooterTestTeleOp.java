package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/** Initializes and tests only the shooter motor. */
@TeleOp(name = "Test: Shooter", group = "Diagnostics")
public final class ShooterTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);

    telemetry.addLine("Hold X on either gamepad to run the shooter.");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    waitForStart();

    try {
      while (opModeIsActive()) {
        shooter.setActive(gamepad1.x || gamepad2.x);
        telemetry.addData("Shooter", shooter.isActive() ? "ON" : "OFF");
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      shooter.stop();
    }
  }
}
