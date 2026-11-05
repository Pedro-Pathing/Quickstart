package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;
import androidx.annotation.Nullable;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Pure passive manual calibration of the turret's true mechanical center and travel limits.
 *
 * <p>Servos are completely unpowered (0 power). The human operator physically turns the turret by
 * hand to each hard stop and presses [A] to capture the analog encoder voltage.
 */
@Configurable
@TeleOp(name = "Turret Center Calibration", group = "Tuning")
public class TurretCenterCalibrationOpMode extends OpMode {

  /** Degrees of margin held back from each measured hard stop when recommending travel limits. */
  public static double limitSafetyMarginDeg = 3.0;

  /** Total mechanical travel in degrees, if known. 0 = derive nothing, keep config scale. */
  public static double knownTotalTravelDeg = 0.0;

  private enum State {
    SET_POSITIVE_STOP,
    SET_NEGATIVE_STOP,
    SET_CENTER,
    COMPLETE
  }

  private @Nullable CRServo turnServo;
  private @Nullable AnalogInput turnAnalog;
  private @Nullable String hardwareError = null;

  private State state = State.SET_POSITIVE_STOP;

  private double positiveStopVoltage = Double.NaN;
  private double negativeStopVoltage = Double.NaN;
  private double centerVoltage = Double.NaN;

  private boolean aPrev = false;
  private boolean bPrev = false;

  @Override
  public void init() {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT
    try {
      turnServo = hardwareMap.get(CRServo.class, "turn");
      turnServo.setPower(0);
    } catch (Exception e) {
      hardwareError = "Missing CRServo 'turn'";
    }
    try {
      turnAnalog = hardwareMap.get(AnalogInput.class, "turnanalog");
    } catch (Exception e) {
      if (hardwareError == null) hardwareError = "Missing AnalogInput 'turnanalog'";
    }

    telemetry.addLine("=== MANUAL (PHYSICAL) TURRET CENTER CALIBRATION ===");
    if (hardwareError != null) {
      telemetry.addLine("ERROR: " + hardwareError);
    } else {
      telemetry.addLine("Servo power is OFF (0.0). Physically rotate the turret by hand.");
      telemetry.addLine("---------------------------------------------------------");
      telemetry.addLine("1. Physically turn turret BY HAND to POSITIVE hard stop.");
      telemetry.addLine("2. Press [A] to capture POSITIVE stop.");
      telemetry.addLine("3. Physically turn turret BY HAND to NEGATIVE hard stop.");
      telemetry.addLine("4. Press [A] to capture NEGATIVE stop.");
    }
    telemetry.update();
  }

  @Override
  public void start() {
    if (turnServo != null) {
      turnServo.setPower(0);
    }
  }

  @SuppressLint("DefaultLocale")
  @Override
  public void loop() {
    AnalogInput analog = turnAnalog;
    if (hardwareError != null || analog == null) {
      telemetry.addLine("=== CANNOT RUN ===");
      telemetry.addLine(hardwareError != null ? hardwareError : "Missing AnalogInput 'turnanalog'");
      telemetry.update();
      return;
    }

    // Ensure servo remains completely unpowered so operator can rotate freely by hand
    if (turnServo != null) {
      turnServo.setPower(0);
    }

    double voltage = analog.getVoltage();

    boolean aJustPressed = gamepad1.a && !aPrev;
    aPrev = gamepad1.a;

    boolean bJustPressed = gamepad1.b && !bPrev;
    bPrev = gamepad1.b;

    if (bJustPressed) {
      // Reset calibration steps
      state = State.SET_POSITIVE_STOP;
      positiveStopVoltage = Double.NaN;
      negativeStopVoltage = Double.NaN;
    }

    switch (state) {
      case SET_POSITIVE_STOP:
        telemetry.addLine("=== STEP 1: SET POSITIVE HARD STOP ===");
        telemetry.addLine("Physically turn turret BY HAND to POSITIVE hard stop.");
        telemetry.addLine("Press [A] (Cross) when aligned at POSITIVE stop.");
        telemetry.addLine("");
        telemetry.addData("Live Voltage", String.format("%.4f V", voltage));
        telemetry.addLine("Servo Power: 0.0 (FREE ROTATION)");

        if (aJustPressed) {
          positiveStopVoltage = voltage;
          state = State.SET_NEGATIVE_STOP;
        }
        break;

      case SET_NEGATIVE_STOP:
        telemetry.addLine("=== STEP 2: SET NEGATIVE HARD STOP ===");
        telemetry.addLine("Physically turn turret BY HAND to NEGATIVE hard stop.");
        telemetry.addLine("Press [A] (Cross) when aligned at NEGATIVE stop.");
        telemetry.addLine("");
        telemetry.addData("Live Voltage", String.format("%.4f V", voltage));
        telemetry.addData("Captured Positive Stop", String.format("%.4f V", positiveStopVoltage));
        telemetry.addLine("Servo Power: 0.0 (FREE ROTATION)");

        if (aJustPressed) {
          negativeStopVoltage = voltage;
          state = State.SET_CENTER;
        }
        break;

      case SET_CENTER:
        telemetry.addLine("=== STEP 3: SET APPROXIMATE CENTER ===");
        telemetry.addLine("Turn the turret BY HAND to roughly the middle of its travel.");
        telemetry.addLine("Press [A] when there.");
        telemetry.addLine("");
        telemetry.addLine("Needed because the potentiometer wraps: the two stops sit on a circle,");
        telemetry.addLine("so there are TWO arcs between them and the stop voltages alone cannot");
        telemetry.addLine("say which one the turret actually travels. Rough is fine -- this only");
        telemetry.addLine("picks the arc; the precise center is computed from the stops.");
        telemetry.addLine("");
        telemetry.addData("Live Voltage", String.format("%.4f V", voltage));
        telemetry.addData("Captured Positive Stop", String.format("%.4f V", positiveStopVoltage));
        telemetry.addData("Captured Negative Stop", String.format("%.4f V", negativeStopVoltage));

        if (aJustPressed) {
          centerVoltage = voltage;
          state = State.COMPLETE;
        }
        break;

      case COMPLETE:
        reportResults(bJustPressed);
        break;
    }

    if (state != State.COMPLETE) {
      telemetry.addLine("");
      telemetry.addLine("[B] (Circle) -> Reset / Start Over");
    }

    telemetry.update();
  }

  /** Shortest signed distance from {@code from} to {@code to} on the potentiometer's circle. */
  private static double circularDelta(double from, double to, double fullScale) {
    double d = to - from;
    if (fullScale > 0) {
      double half = fullScale / 2.0;
      d -= Math.floor((d + half) / fullScale) * fullScale;
    }
    return d;
  }

  @SuppressLint("DefaultLocale")
  private void reportResults(boolean bJustPressed) {
    double fullScale = config.turret.analog_encoder.full_scale_voltage;

    // Walk from the negative stop to the positive stop along the arc that actually contains the
    // operator-marked center. Taking a plain (min+max)/2 midpoint silently picks whichever arc
    // happens to not cross the wrap -- which on this robot was the wrong one, putting the computed
    // zero on the arc the turret never travels and reading -70 deg at true center.
    double toCenter = circularDelta(negativeStopVoltage, centerVoltage, fullScale);
    double toPositive = circularDelta(negativeStopVoltage, positiveStopVoltage, fullScale);

    // If the center does not lie between the stops along this arc, we picked the wrong direction.
    if (fullScale > 0 && Math.signum(toCenter) != Math.signum(toPositive)) {
      toPositive += Math.signum(toCenter) * fullScale;
    }

    double spanVolts = Math.abs(toPositive);
    double center = negativeStopVoltage + toPositive / 2.0;
    if (fullScale > 0) {
      center -= Math.floor(center / fullScale) * fullScale;
    }

    telemetry.addLine("=== CALIBRATION COMPLETE ===");
    telemetry.addData("Positive Stop", String.format("%.4f V", positiveStopVoltage));
    telemetry.addData("Negative Stop", String.format("%.4f V", negativeStopVoltage));
    telemetry.addData("Marked Center", String.format("%.4f V", centerVoltage));
    telemetry.addData("Computed Center Voltage", String.format("%.4f V", center));
    telemetry.addData("Total Span", String.format("%.4f V", spanVolts));
    if (fullScale > 0 && Math.abs(circularDelta(0, spanVolts, fullScale)) != spanVolts) {
      telemetry.addLine("NOTE: travel crosses the potentiometer wrap point.");
    }

    double degPerVolt = config.turret.analog_encoder.degrees_per_volt;
    if (knownTotalTravelDeg > 0.0 && spanVolts > 1e-6) {
      degPerVolt = knownTotalTravelDeg / spanVolts;
      telemetry.addLine(
          String.format("Derived scale from known %.1f deg travel", knownTotalTravelDeg));
    } else {
      telemetry.addLine("Using existing degrees_per_volt (set knownTotalTravelDeg to re-derive)");
    }

    double halfTravelDeg = (spanVolts / 2.0) * degPerVolt;
    double usableHalf = Math.max(0.0, halfTravelDeg - limitSafetyMarginDeg);

    telemetry.addLine("");
    telemetry.addLine("--- PASTE INTO config.yaml ---");
    telemetry.addLine("turret:");
    telemetry.addLine("  travel:");
    telemetry.addLine(String.format("    min_angle: %.1f", -usableHalf));
    telemetry.addLine(String.format("    max_angle: %.1f", usableHalf));
    telemetry.addLine("  analog_encoder:");
    telemetry.addLine(String.format("    zero_voltage: %.4f", center));
    telemetry.addLine(String.format("    degrees_per_volt: %.2f", degPerVolt));
    telemetry.addLine(
        String.format("    min_voltage: %.3f", Math.min(negativeStopVoltage, positiveStopVoltage)));
    telemetry.addLine(
        String.format("    max_voltage: %.3f", Math.max(negativeStopVoltage, positiveStopVoltage)));
    telemetry.addLine(String.format("    full_scale_voltage: %.2f", fullScale));
    telemetry.addLine("");
    telemetry.addLine(
        String.format(
            "Total mechanical travel: %.1f deg (+/- %.1f)", halfTravelDeg * 2.0, halfTravelDeg));
    telemetry.addLine("");
    telemetry.addLine("[B] (Circle) -> Re-run / Start Over");

    if (bJustPressed) {
      state = State.SET_POSITIVE_STOP;
      positiveStopVoltage = Double.NaN;
      negativeStopVoltage = Double.NaN;
      centerVoltage = Double.NaN;
    }
  }
}
