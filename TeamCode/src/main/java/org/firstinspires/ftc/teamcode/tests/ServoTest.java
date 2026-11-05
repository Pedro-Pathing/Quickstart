package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends OpMode {

  private CRServo turnServo;

  @Override
  public void init() {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    org.firstinspires.ftc.teamcode.utilities.OpModeUtil.reportConfig(telemetry);
    turnServo = hardwareMap.get(CRServo.class, "turn");
    telemetry.addData("Status", "Initialized - Use gamepad1 triggers to control servo");
    telemetry.addData("Left Trigger", "Reverse");
    telemetry.addData("Right Trigger", "Forward");
    telemetry.update();
  }

  @Override
  public void loop() {
    double power = 0;

    if (gamepad1.right_trigger > 0.1) {
      power = gamepad1.right_trigger;
      turnServo.setDirection(CRServo.Direction.FORWARD);
    } else if (gamepad1.left_trigger > 0.1) {
      power = gamepad1.left_trigger;
      turnServo.setDirection(CRServo.Direction.REVERSE);
    }

    // Never more than the turret's own limit: a wiring check should not slam the travel stops at
    // full power.
    turnServo.setPower(
        power
            * org.firstinspires.ftc.teamcode.robot.config.generated.config.turret.max_power_output);

    telemetry.addData("Servo Power", String.format("%.2f", power));
    telemetry.addData(
        "Direction",
        gamepad1.right_trigger > 0.1
            ? "FORWARD"
            : gamepad1.left_trigger > 0.1 ? "REVERSE" : "STOPPED");
    telemetry.addData("Instructions", "Pull triggers to test servo movement");
    telemetry.update();
  }
}
