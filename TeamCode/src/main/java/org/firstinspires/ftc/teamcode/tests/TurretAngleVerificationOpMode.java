package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;
import androidx.annotation.Nullable;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

@Configurable
@TeleOp(name = "Turret Angle Verification", group = "Tuning")
public class TurretAngleVerificationOpMode extends OpMode {

  // Bylazar Dashboard Tunable parameters
  public static double p;
  public static double i;
  public static double d;
  public static double f;
  public static double maxPower;
  public static double targetAngle = 0.0;

  private Turret turret;
  private @Nullable IMU turnIMU;
  private @Nullable AnalogInput turnAnalog;
  private TelemetryManager telemetryM;

  private double prevTargetAngle = 0.0;
  private boolean prevActiveHolding = false;
  private double startImuAngle = 0.0;

  private final ElapsedTime settleTimer = new ElapsedTime();
  private boolean isSettled = false;
  private double settlingTimeSec = -1.0;

  private boolean dpadUpPrev = false;
  private boolean dpadDownPrev = false;
  private boolean dpadLeftPrev = false;
  private boolean dpadRightPrev = false;

  @Override
  public void init() {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT

    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    turret = new Turret(hardwareMap, telemetry);
    turret.setTargetPoint(100.0, 100.0);

    // Seed Bylazar static fields from config defaults. These fields have no initializers, so at
    // class-load time (when Panels scans @Configurable classes) they all read 0 -- which is what
    // the dashboard would keep showing, and keep pushing back, unless we tell it they changed.
    p = turret.getP();
    i = turret.getI();
    d = turret.getD();
    f = turret.getF();
    maxPower = turret.getMaxPower();
    PanelsConfigurables.INSTANCE.refreshClass(this);

    try {
      turnAnalog = hardwareMap.get(AnalogInput.class, "turnanalog");
    } catch (Exception ignored) {
    }

    try {
      turnIMU = hardwareMap.get(IMU.class, "turnImu");
      RevHubOrientationOnRobot.LogoFacingDirection logoDir =
          RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
      RevHubOrientationOnRobot.UsbFacingDirection usbDir =
          RevHubOrientationOnRobot.UsbFacingDirection.UP;

      if (config.turret != null && config.turret.orientation != null) {
        try {
          logoDir =
              RevHubOrientationOnRobot.LogoFacingDirection.valueOf(config.turret.orientation.logo);
          usbDir =
              RevHubOrientationOnRobot.UsbFacingDirection.valueOf(config.turret.orientation.usb);
        } catch (Exception ignored) {
        }
      }

      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(logoDir, usbDir));
      turnIMU.initialize(parameters);
      turnIMU.resetYaw();
    } catch (Exception e) {
      telemetry.addData("IMU Error", e.getMessage());
    }

    startImuAngle = getRawImuAngle();

    telemetry.addLine("=== TURRET ANGLE VERIFICATION ===");
    // Catches a bad pushed zero_voltage before it drives the turret into a discontinuity.
    String encoderWarning = turret.getEncoderRangeWarning();
    if (encoderWarning != null) {
      telemetry.addLine("*** ENCODER CONFIG WARNING ***");
      telemetry.addLine(encoderWarning);
      telemetry.addLine("");
    }
    telemetry.addData(
        "Encoder Angle At Init", String.format("%.2f°", turret.getCurrentTurnAngle()));
    telemetry.addLine("Compares Analog Encoder angle against IMU (Ground Truth).");
    telemetry.addLine("Controls:");
    telemetry.addLine("  D-Pad UP / DOWN : +/- 5° target angle");
    telemetry.addLine("  D-Pad LEFT / RIGHT : +/- 15° target angle");
    telemetry.addLine("  Button [Y] : Set target to 0° (Center)");
    telemetry.addLine("  Button [X] : Set target to +30°");
    telemetry.addLine("  Button [B] : Set target to -30°");
    telemetry.addLine("  Button [A] : Zero IMU Ground Truth reference");
    telemetry.update();
  }

  private boolean activeHolding = false;

  @Override
  public void start() {
    activeHolding = false;
    turret.setAimMode(Turret.AimMode.IDLE);
    turret.periodic();
    settleTimer.reset();
    isSettled = false;
    settlingTimeSec = -1.0;
  }

  private double getRawImuAngle() {
    if (turnIMU == null) return 0.0;
    YawPitchRollAngles orientation = turnIMU.getRobotYawPitchRollAngles();
    return orientation != null ? Math.toDegrees(orientation.getYaw(AngleUnit.RADIANS)) : 0.0;
  }

  private boolean aPrev = false;
  private boolean bPrev = false;
  private boolean xPrev = false;
  private boolean yPrev = false;

  @SuppressLint("DefaultLocale")
  @Override
  public void loop() {
    // Sync Bylazar dashboard PIDF & max power values to turret subsystem
    turret.setPIDF(p, i, d, f);
    turret.setMaxPower(maxPower);

    // Gamepad controls for target angle
    if (gamepad1.dpad_up && !dpadUpPrev) {
      targetAngle += 5.0;
      activeHolding = true;
    }
    if (gamepad1.dpad_down && !dpadDownPrev) {
      targetAngle -= 5.0;
      activeHolding = true;
    }
    if (gamepad1.dpad_left && !dpadLeftPrev) {
      targetAngle += 15.0;
      activeHolding = true;
    }
    if (gamepad1.dpad_right && !dpadRightPrev) {
      targetAngle -= 15.0;
      activeHolding = true;
    }

    dpadUpPrev = gamepad1.dpad_up;
    dpadDownPrev = gamepad1.dpad_down;
    dpadLeftPrev = gamepad1.dpad_left;
    dpadRightPrev = gamepad1.dpad_right;

    if (gamepad1.y && !yPrev) {
      targetAngle = 0.0;
      activeHolding = true;
    }
    if (gamepad1.x && !xPrev) {
      targetAngle = 30.0;
      activeHolding = true;
    }
    if (gamepad1.b && !bPrev) {
      targetAngle = -30.0;
      activeHolding = true;
    }

    if (gamepad1.a && !aPrev) {
      targetAngle = 0.0;
      if (turnIMU != null) turnIMU.resetYaw();
      startImuAngle = getRawImuAngle();
      activeHolding = true;
    }

    yPrev = gamepad1.y;
    xPrev = gamepad1.x;
    bPrev = gamepad1.b;
    aPrev = gamepad1.a;

    // Left Trigger releases active holding (turns off servo power / IDLE)
    if (gamepad1.left_trigger > 0.2) {
      activeHolding = false;
    }

    // Right Bumper clears a latched stall. Deliberate, and it drops out of closed loop first so
    // clearing the fault cannot immediately resume pushing against whatever caused it.
    if (gamepad1.right_bumper) {
      activeHolding = false;
      turret.clearFault();
    }

    // Target angle safety clamp
    targetAngle = Math.clamp(targetAngle, -45.0, 45.0);

    // Detect target change or state transition to reset settling timer
    boolean targetChanged =
        Math.abs(targetAngle - prevTargetAngle) > 1e-4 || (activeHolding && !prevActiveHolding);
    if (targetChanged) {
      settleTimer.reset();
      isSettled = false;
      settlingTimeSec = -1.0;
    }
    prevTargetAngle = targetAngle;
    prevActiveHolding = activeHolding;

    // Apply target angle or stay idle
    if (activeHolding) {
      turret.setHoldAngle(targetAngle);
      turret.setAimMode(Turret.AimMode.HOLD);
    } else {
      turret.setAimMode(Turret.AimMode.IDLE);
    }
    turret.periodic();

    // Measure values
    double rawVoltage = turnAnalog != null ? turnAnalog.getVoltage() : 0.0;
    double analogCalculatedAngle = turret.getCurrentTurnAngle();
    double imuGroundTruthAngle = AngleUnit.normalizeDegrees(getRawImuAngle() - startImuAngle);
    double targetVsMeasuredError = targetAngle - analogCalculatedAngle;
    double encoderVsImuError = Math.abs(analogCalculatedAngle - imuGroundTruthAngle);

    // Track settling time
    if (activeHolding && !isSettled && turret.isTurnDone()) {
      isSettled = true;
      settlingTimeSec = settleTimer.seconds();
    }

    // Bylazar Telemetry for Panels Dashboard
    if (telemetryM != null) {
      telemetryM.addData("Target Angle (deg)", targetAngle);
      telemetryM.addData("Measured Angle Analog (deg)", analogCalculatedAngle);
      telemetryM.addData("Measured Angle IMU (deg)", imuGroundTruthAngle);
      telemetryM.addData("Target vs Measured Error (deg)", targetVsMeasuredError);
      telemetryM.addData("Encoder vs IMU Error (deg)", encoderVsImuError);
      telemetryM.addData("Is Settled", isSettled);
      telemetryM.addData("Settling Time (sec)", isSettled ? settlingTimeSec : -1.0);
      telemetryM.addData("Turret P", p);
      telemetryM.addData("Turret I", i);
      telemetryM.addData("Turret D", d);
      telemetryM.addData("Turret F", f);
      telemetryM.addData("Turret Max Power", maxPower);
      telemetryM.update();
    }

    // FTC Driver Station Telemetry
    telemetry.addLine("=== ANGLE & SETTLING VERIFICATION ===");
    telemetry.addData("Servo Status", activeHolding ? "HOLDING AT TARGET" : "IDLE (POWER OFF)");
    telemetry.addData("Commanded Target", String.format("%.1f°", targetAngle));
    telemetry.addData("Analog Encoder (Measured)", String.format("%.2f°", analogCalculatedAngle));
    telemetry.addData("Target vs Measured Error", String.format("%.2f°", targetVsMeasuredError));
    telemetry.addData(
        "Settling Time",
        isSettled
            ? String.format("%.3f s", settlingTimeSec)
            : (activeHolding ? "Settling..." : "N/A"));
    telemetry.addLine("--- GROUND TRUTH & SENSOR DATA ---");
    telemetry.addData("IMU Ground Truth", String.format("%.2f°", imuGroundTruthAngle));
    telemetry.addData("Encoder vs IMU Error", String.format("%.2f°", encoderVsImuError));
    if (turret.isFaulted()) {
      telemetry.addLine("*** SAFETY FAULT - power cut. [Right Bumper] to clear. ***");
      telemetry.addLine(turret.getFaultReason());
    }
    telemetry.addData(
        "Power Blocked",
        turret.getPowerBlockedReason() == null ? "no" : turret.getPowerBlockedReason());
    telemetry.addData("Encoder Within Travel", turret.isReadingWithinTravel());
    telemetry.addData("Raw Analog Voltage", String.format("%.4f V", rawVoltage));
    telemetry.addData("Zero Voltage (Turret)", String.format("%.4f V", turret.getZeroVoltage()));
    telemetry.addData(
        "Deg/Volt (Config)", String.format("%.2f", config.turret.analog_encoder.degrees_per_volt));
    telemetry.addLine("");
    telemetry.addLine("Press [Left Trigger] to kill servo power / IDLE");
    telemetry.update();
  }
}
