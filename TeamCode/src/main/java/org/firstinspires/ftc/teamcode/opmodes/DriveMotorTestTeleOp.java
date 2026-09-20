package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive.MotorPosition;

/** Runs one drivetrain motor at a time at reduced power. Lift the robot before using. */
@TeleOp(name = "Test: Drive Motors", group = "Diagnostics")
public final class DriveMotorTestTeleOp extends LinearOpMode {
  private static final double STICK_DEADBAND = 0.05;

  @Override
  public void runOpMode() {
    MecanumDrive drive = new MecanumDrive(hardwareMap);
    MotorPosition selectedMotor = MotorPosition.FRONT_LEFT;

    telemetry.addLine("Lift the wheels off the floor before running this test.");
    telemetry.addLine("D-pad selects a motor; left-stick Y powers only that motor.");
    telemetry.update();
    waitForStart();

    try {
      while (opModeIsActive()) {
        selectedMotor = readSelection(selectedMotor);
        double power = readTestPower();
        drive.runSingleMotor(selectedMotor, power);

        telemetry.addData("Selected Motor", selectedMotor);
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("D-pad Up", "frontLeft");
        telemetry.addData("D-pad Right", "frontRight");
        telemetry.addData("D-pad Left", "backLeft");
        telemetry.addData("D-pad Down", "backRight");
        telemetry.update();
        idle();
      }
    } finally {
      drive.stop();
    }
  }

  private MotorPosition readSelection(MotorPosition current) {
    if (gamepad1.dpad_up || gamepad2.dpad_up) {
      return MotorPosition.FRONT_LEFT;
    }
    if (gamepad1.dpad_right || gamepad2.dpad_right) {
      return MotorPosition.FRONT_RIGHT;
    }
    if (gamepad1.dpad_left || gamepad2.dpad_left) {
      return MotorPosition.BACK_LEFT;
    }
    if (gamepad1.dpad_down || gamepad2.dpad_down) {
      return MotorPosition.BACK_RIGHT;
    }
    return current;
  }

  private double readTestPower() {
    double stickY =
        Math.abs(gamepad1.left_stick_y) > STICK_DEADBAND
            ? gamepad1.left_stick_y
            : gamepad2.left_stick_y;
    if (Math.abs(stickY) <= STICK_DEADBAND) {
      return 0;
    }
    return -stickY * RobotConfig.Diagnostics.DRIVE_MOTOR_TEST_MAX_POWER;
  }
}
