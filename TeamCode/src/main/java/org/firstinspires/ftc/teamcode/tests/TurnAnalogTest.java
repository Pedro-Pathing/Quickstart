package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Turn Analog Test", group = "Test")
public class TurnAnalogTest extends OpMode {

  private AnalogInput turnAnalog;
  private CRServo turnServo;
  private double minVoltage = 3.3;
  private double maxVoltage = 0.0;

  @Override
  public void init() {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    org.firstinspires.ftc.teamcode.utilities.OpModeUtil.reportConfig(telemetry);

    try {
      turnAnalog = hardwareMap.get(AnalogInput.class, "turnanalog");
    } catch (Exception ignored) {
    }

    try {
      turnServo = hardwareMap.get(CRServo.class, "turn");
    } catch (Exception ignored) {
    }

    telemetry.addLine("=== TURN ANALOG SERVO TESTER ===");
    if (turnAnalog != null) {
      telemetry.addLine("AnalogInput 'turnanalog' connected");
    } else {
      telemetry.addLine("ERROR: AnalogInput 'turnanalog' NOT FOUND in Hardware Map!");
    }
    if (turnServo != null) {
      telemetry.addLine("CRServo 'turn' detected - pull triggers to drive servo");
    } else {
      telemetry.addLine("No CRServo 'turn' found - manual measurement mode");
    }
    telemetry.addLine("Press [A] to reset min/max voltage range");
    telemetry.update();
  }

  @SuppressLint("DefaultLocale")
  @Override
  public void loop() {
    if (turnAnalog == null) {
      telemetry.addLine("=== ERROR: MISSING ANALOG INPUT ===");
      telemetry.addLine("Configure AnalogInput 'turnanalog' on Expansion Hub!");
      telemetry.update();
      return;
    }
    double rawVoltage = turnAnalog.getVoltage();
    double normalizedPosition = rawVoltage / 3.3;
    double degrees = normalizedPosition * 360.0;

    // Track min/max observed voltage
    if (rawVoltage < minVoltage) minVoltage = rawVoltage;
    if (rawVoltage > maxVoltage) maxVoltage = rawVoltage;

    if (gamepad1.a) {
      minVoltage = rawVoltage;
      maxVoltage = rawVoltage;
    }

    // Drive CRServo if connected
    double power = 0;
    if (turnServo != null) {
      if (gamepad1.right_trigger > 0.05) {
        power = gamepad1.right_trigger;
        turnServo.setDirection(CRServo.Direction.FORWARD);
      } else if (gamepad1.left_trigger > 0.05) {
        power = gamepad1.left_trigger;
        turnServo.setDirection(CRServo.Direction.REVERSE);
      }
      // Never more than the turret's own limit, so watching the encoder does not slam the stops.
      turnServo.setPower(
          power
              * org.firstinspires
                  .ftc
                  .teamcode
                  .robot
                  .config
                  .generated
                  .config
                  .turret
                  .max_power_output);
    }

    telemetry.addLine("=== ANALOG INPUT: 'turnanalog' ===");
    telemetry.addData("Raw Voltage", String.format("%.3f V / 3.300 V", rawVoltage));
    telemetry.addData(
        "Normalized Position", String.format("%.4f (0.0 to 1.0)", normalizedPosition));
    telemetry.addData("Angle (Degrees)", String.format("%.2f° (0 to 360°)", degrees));
    telemetry.addLine("");
    telemetry.addData("Min Voltage Observed", String.format("%.3f V", minVoltage));
    telemetry.addData("Max Voltage Observed", String.format("%.3f V", maxVoltage));
    telemetry.addLine("");

    if (turnServo != null) {
      telemetry.addData("Servo Power", String.format("%.2f", power));
    }
    telemetry.addLine("Controls: [A] Reset Min/Max Range | Triggers: Move Servo");
    telemetry.update();
  }
}
