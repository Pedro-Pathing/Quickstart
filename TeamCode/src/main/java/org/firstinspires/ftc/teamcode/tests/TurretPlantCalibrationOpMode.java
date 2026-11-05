package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Measures the turret's plant characteristics and derives a proportional gain from them, replacing
 * relay autotuning for this mechanism.
 *
 * <p>Relay (Ziegler-Nichols / Astrom-Hagglund) autotuning assumes a self-regulating plant whose
 * output settles when the input is held. A CRServo driving a position is an <em>integrator</em>:
 * power commands angular velocity, so holding any non-zero power ramps the angle forever. Under
 * relay feedback an integrator still oscillates, but the period of that oscillation is set by the
 * control loop's own switching delay rather than by the mechanism -- so the "ultimate period" it
 * reports is roughly the loop rate, and the gains derived from it are meaningless no matter how
 * cleanly the relay runs. That is why {@code TurretAutotuneOpMode} never produced usable numbers
 * here.
 *
 * <p>What an integrator needs instead is its gain. This OpMode measures two things directly:
 *
 * <ol>
 *   <li><b>kS</b> -- the power at which the turret first breaks static friction. This is {@code
 *       turret.feed_forward}.
 *   <li><b>K</b> -- the plant gain in degrees/second of turret motion per unit of commanded power
 *       above kS.
 * </ol>
 *
 * <p>With those, proportional control of an integrator has a first-order closed-loop response whose
 * time constant is {@code tau = 1 / (K * p)}. Pick how fast you want the turret to converge and the
 * gain follows: {@code p = 1 / (K * tau)}. No oscillation required, and nothing is driven into a
 * hard stop to get there.
 */
@Configurable
@TeleOp(name = "Turret Plant Calibration", group = "Tuning")
public class TurretPlantCalibrationOpMode extends LinearOpMode {

  /** How quickly power is ramped while hunting for the stiction breakaway point (power/second). */
  public static double stictionRampRate = 0.05;

  /** Degrees of motion that count as "it started moving" during the stiction ramp. */
  public static double breakawayMoveDeg = 1.5;

  /** Powers sampled when measuring plant gain. Must be above the measured kS to be useful. */
  public static double gainTestPowerLow = 0.18;

  public static double gainTestPowerMid = 0.28;
  public static double gainTestPowerHigh = 0.40;

  /** Seconds to let the turret reach steady speed before timing it. */
  public static double settleSeconds = 0.20;

  /** Seconds over which angular velocity is measured. */
  public static double measureSeconds = 0.35;

  /** Desired closed-loop time constant (seconds). Smaller = snappier, more overshoot risk. */
  public static double desiredTimeConstantSec = 0.20;

  /** Degrees of clearance kept from the travel limits while manoeuvring. */
  public static double safetyMarginDeg = 4.0;

  private Turret turret;
  private final ElapsedTime timer = new ElapsedTime();

  /**
   * @param power magnitude of the commanded power
   * @param degPerSec magnitude of the resulting speed
   * @param signAgrees whether the turret moved in the direction that was commanded. A single
   *     disagreement means the control loop is inverted and will run away from its target.
   */
  private record GainSample(double power, double degPerSec, boolean signAgrees) {}

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT
    for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    turret = new Turret(hardwareMap, telemetry);
    turret.setAimMode(Turret.AimMode.MANUAL);

    telemetry.addLine("=== TURRET PLANT CALIBRATION ===");
    telemetry.addLine("Measures stiction (kS) and plant gain (K), then derives p.");
    telemetry.addLine("The turret will move on its own within its travel limits.");
    telemetry.addLine("");
    telemetry.addLine("Press START to begin, [B] at any time to abort.");
    telemetry.update();

    waitForStart();
    if (isStopRequested()) return;

    try {
      double ksPositive = measureBreakaway(+1.0);
      if (!opModeIsActive()) return;
      double ksNegative = measureBreakaway(-1.0);
      if (!opModeIsActive()) return;
      double kS = (ksPositive + Math.abs(ksNegative)) / 2.0;

      List<GainSample> samples = new ArrayList<>();
      double[] powers = {gainTestPowerLow, gainTestPowerMid, gainTestPowerHigh};
      double direction = +1.0;
      for (double power : powers) {
        if (!opModeIsActive()) return;
        direction = chooseDirectionWithRoom(direction);
        double commanded = power * direction;
        double degPerSec = measureSpeed(commanded);
        // Keep the magnitude for the gain fit, but hold on to whether the turret actually moved
        // the way it was told -- that is the one thing this OpMode can prove that no amount of
        // closed-loop tuning can, and discarding it hid an inverted loop.
        samples.add(
            new GainSample(
                power,
                Math.abs(degPerSec),
                Math.abs(degPerSec) < 1e-6 || Math.signum(degPerSec) == Math.signum(commanded)));
        direction = -direction;
      }

      stopTurret();
      reportResults(ksPositive, ksNegative, kS, samples);
    } finally {
      stopTurret();
    }

    while (opModeIsActive() && !isStopRequested()) {
      idle();
    }
  }

  private void clearCache() {
    for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
      module.clearBulkCache();
    }
  }

  /** Named stopTurret rather than stop to avoid clashing with OpMode.stop(). */
  private void stopTurret() {
    turret.setManualPower(0);
    turret.periodic();
  }

  private void drive(double power) {
    turret.setManualPower(power);
    turret.periodic();
  }

  private boolean hasRoom(double direction) {
    var travel = config.turret.travel;
    double angle = turret.getCurrentTurnAngle();
    return direction > 0
        ? angle < travel.max_angle - safetyMarginDeg
        : angle > travel.min_angle + safetyMarginDeg;
  }

  /** Returns a direction with travel available, preferring the requested one. */
  private double chooseDirectionWithRoom(double preferred) {
    return hasRoom(preferred) ? preferred : -preferred;
  }

  /** Ramps power from zero until the turret breaks free; returns the signed breakaway power. */
  private double measureBreakaway(double direction) {
    direction = chooseDirectionWithRoom(direction);
    clearCache();
    double startAngle = turret.getCurrentTurnAngle();
    double power = 0.0;
    timer.reset();
    double last = timer.seconds();

    while (opModeIsActive() && !gamepad1.b && power < config.turret.max_power_output) {
      clearCache();
      double now = timer.seconds();
      power += stictionRampRate * (now - last);
      last = now;

      drive(power * direction);

      double moved = Math.abs(turret.getCurrentTurnAngle() - startAngle);
      telemetry.addLine("=== MEASURING STICTION (kS) ===");
      telemetry.addData("Direction", direction > 0 ? "POSITIVE" : "NEGATIVE");
      telemetry.addData("Power", String.format("%.4f", power));
      telemetry.addData("Moved", String.format("%.2f deg", moved));
      telemetry.update();

      if (moved >= breakawayMoveDeg) break;
      if (!hasRoom(direction)) break;
    }
    stopTurret();
    sleep(250);
    return power * direction;
  }

  /** Drives at a fixed power and returns the resulting steady angular velocity (deg/s). */
  private double measureSpeed(double signedPower) {
    clearCache();
    timer.reset();
    while (opModeIsActive() && !gamepad1.b && timer.seconds() < settleSeconds) {
      clearCache();
      drive(signedPower);
      telemetry.addLine("=== MEASURING PLANT GAIN (K) ===");
      telemetry.addData("Power", String.format("%.3f", signedPower));
      telemetry.addData("Phase", "settling");
      telemetry.update();
      if (!hasRoom(Math.signum(signedPower))) break;
    }

    clearCache();
    double startAngle = turret.getCurrentTurnAngle();
    double startTime = timer.seconds();
    while (opModeIsActive()
        && !gamepad1.b
        && timer.seconds() - startTime < measureSeconds
        && hasRoom(Math.signum(signedPower))) {
      clearCache();
      drive(signedPower);
      telemetry.addLine("=== MEASURING PLANT GAIN (K) ===");
      telemetry.addData("Power", String.format("%.3f", signedPower));
      telemetry.addData("Phase", "timing");
      telemetry.update();
    }

    clearCache();
    double elapsed = timer.seconds() - startTime;
    double travelled = turret.getCurrentTurnAngle() - startAngle;
    stopTurret();
    sleep(250);
    return elapsed > 1e-6 ? travelled / elapsed : 0.0;
  }

  @SuppressLint("DefaultLocale")
  private void reportResults(
      double ksPositive, double ksNegative, double kS, List<GainSample> samples) {
    // Least-squares slope of degPerSec against power, forced through the stiction intercept:
    // velocity ~= K * (power - kS), so K = sum(v * (p - kS)) / sum((p - kS)^2).
    double num = 0;
    double den = 0;
    for (GainSample s : samples) {
      double x = s.power() - kS;
      num += s.degPerSec() * x;
      den += x * x;
    }
    double k = den > 1e-9 ? num / den : 0.0;
    double recommendedP = (k > 1e-9) ? 1.0 / (k * desiredTimeConstantSec) : 0.0;

    boolean loopInverted = samples.stream().anyMatch(s -> !s.signAgrees());

    telemetry.addLine("=== RESULTS ===");
    if (loopInverted) {
      telemetry.addLine("*** CONTROL LOOP IS INVERTED ***");
      telemetry.addLine("The turret moved OPPOSITE the commanded direction. Closed-loop control");
      telemetry.addLine("will drive away from the target and into a hard stop. Flip");
      telemetry.addLine("turret.servo_direction_inverted in config.yaml and re-run BEFORE");
      telemetry.addLine("using any closed-loop OpMode. The gains below are still valid.");
      telemetry.addLine("");
    } else {
      telemetry.addLine("Loop sign OK: turret moves the way it is commanded.");
    }
    telemetry.addData("kS positive", String.format("%.4f", ksPositive));
    telemetry.addData("kS negative", String.format("%.4f", ksNegative));
    telemetry.addData("kS (mean)", String.format("%.4f", kS));
    telemetry.addLine("");
    for (GainSample s : samples) {
      telemetry.addLine(String.format("  power %.3f -> %.1f deg/s", s.power(), s.degPerSec()));
    }
    telemetry.addData("Plant gain K", String.format("%.1f deg/s per power", k));
    telemetry.addLine("");
    telemetry.addLine("--- PASTE INTO config.yaml ---");
    telemetry.addLine("turret:");
    telemetry.addLine(String.format("  ks_positive: %.4f", Math.abs(ksPositive)));
    telemetry.addLine(String.format("  ks_negative: %.4f", Math.abs(ksNegative)));
    // Velocity feedforward: power per deg/s, so the turret keeps up with a turning chassis.
    telemetry.addLine(String.format("  kv: %.5f", k > 0 ? 1.0 / k : 0.0));
    telemetry.addLine("  pidf:");
    telemetry.addLine(String.format("    p: %.5f", recommendedP));
    telemetry.addLine("    i: 0");
    telemetry.addLine("    d: 0");
    telemetry.addLine("    f: 0");
    telemetry.addLine("");
    telemetry.addLine(
        String.format("p targets a %.2fs closed-loop time constant.", desiredTimeConstantSec));
    telemetry.addLine("Start with i and d at 0: on an integrating plant P+kS is usually enough,");
    telemetry.addLine("and d amplifies analog encoder noise. Add i only for steady-state droop.");
    telemetry.update();
  }
}
