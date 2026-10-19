package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.control.TeleOpControlThreads;
import org.firstinspires.ftc.teamcode.localization.RobotLocalization;
import org.firstinspires.ftc.teamcode.subsystems.GameMechanisms;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.telemetry.LocalizationTelemetry;

@TeleOp(name = "Limelight TeleOp", group = "Localization")
public class LimelightTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    RobotLocalization localization = new RobotLocalization(hardwareMap);
    MecanumDrive drive = new MecanumDrive(hardwareMap);
    GameMechanisms mechanisms = new GameMechanisms(hardwareMap);
    TeleOpControlThreads controlThreads =
        new TeleOpControlThreads(gamepad1, gamepad2, drive, mechanisms);
    LocalizationTelemetry localizationTelemetry = new LocalizationTelemetry();

    localization.resetPose(0, 0, 0);
    localization.start();
    try {
      telemetry.addData("Status", "Initialized. Waiting for start...");
      telemetry.addData("LL Connected", localization.isLimelightConnected());
      OdometryTelemetry.add(telemetry);
      telemetry.update();

      waitForStart();
      controlThreads.start();

      while (opModeIsActive()) {
        Throwable controlFailure = controlThreads.getFailure();
        if (controlFailure != null) {
          throw new IllegalStateException("A TeleOp control thread failed", controlFailure);
        }

        localization.update();
        DriveCommand command = controlThreads.getLatestDriveCommand();

        localizationTelemetry.add(telemetry, localization);
        telemetry.addLine();
        telemetry.addData("Active Driver", "Gamepad %d", command.getGamepadNumber());
        telemetry.addData("Drive", "%.2f", command.getDrive());
        telemetry.addData("Strafe", "%.2f", command.getStrafe());
        telemetry.addData("Rotate", "%.2f", command.getRotate());
        telemetry.addData(
            "Intake",
            "%s / %s",
            mechanisms.isIntakeEnabled() ? "ON" : "OFF",
            mechanisms.getIntakeDirection());
        telemetry.addData("Shooter", mechanisms.isShooterActive() ? "ON" : "OFF");
        telemetry.addData("Servo Motor", mechanisms.isServoEnabled() ? "ON" : "OFF");
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      controlThreads.stop();
      localization.stop();
    }
  }
}
