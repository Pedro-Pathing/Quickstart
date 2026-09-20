package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.DualShooterRpmControls;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/** Independently adjusts and measures the two shooter motor RPM targets. */
@TeleOp(name = "Test: Dual Shooter RPM", group = "Diagnostics")
public final class DualShooterRpmTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
    DualShooterRpmControls controls = new DualShooterRpmControls();

    telemetry.addLine("Both shooter targets start at 6000 RPM after Play.");
    telemetry.addLine("Shoot1: X +100 RPM, A -100 RPM");
    telemetry.addLine("Shoot2: Y +100 RPM, B -100 RPM");
    telemetry.update();
    waitForStart();

    shooter.setTargetRpms(controls.getMotor1Rpm(), controls.getMotor2Rpm());
    shooter.setActive(true);
    try {
      while (opModeIsActive()) {
        controls.update(gamepad1, gamepad2);
        shooter.setTargetRpms(controls.getMotor1Rpm(), controls.getMotor2Rpm());

        double actualRpm1 = shooter.getActualRpm1();
        double actualRpm2 = shooter.getActualRpm2();
        telemetry.addData(
            "Shoot1", "target %.0f, actual %.0f RPM", controls.getMotor1Rpm(), actualRpm1);
        telemetry.addData("Shoot1 Error", "%.0f RPM", controls.getMotor1Rpm() - actualRpm1);
        telemetry.addData(
            "Shoot2", "target %.0f, actual %.0f RPM", controls.getMotor2Rpm(), actualRpm2);
        telemetry.addData("Shoot2 Error", "%.0f RPM", controls.getMotor2Rpm() - actualRpm2);
        telemetry.update();
        idle();
      }
    } finally {
      shooter.stop();
    }
  }
}
