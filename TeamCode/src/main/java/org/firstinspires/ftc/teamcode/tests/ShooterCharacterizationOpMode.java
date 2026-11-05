package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.ballistics.FlywheelFeedforwardFit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Steady-state characterization of the shooter flywheel.
 *
 * <p>The previous {@link ShooterAutotuneOpMode} fit the feedforward from only two points — a
 * stiction ramp for kS and a single full-power step for vMax — then forced the line through {@code
 * (0, kS)} and {@code (vMax, 1.0)}. Any error in either point tilts the whole line, and its kP
 * search could not compensate because a feedforward that is too hot overshoots at every kP.
 *
 * <p>This OpMode instead holds a staircase of open-loop power setpoints, waits for the flywheel to
 * actually settle at each one, and least-squares fits {@code u = kV * v + kS} over all of them. It
 * reports the residuals so a bad fit is visible rather than silent.
 */
@TeleOp(name = "Shooter Characterization", group = "Calibration")
@Configurable
public class ShooterCharacterizationOpMode extends LinearOpMode {

  /** One settled (power, velocity) operating point. */
  /** One settled step; {@code busVolts} is the mean battery voltage while it was sampled. */
  public record SweepPoint(
      double power, double velocityTicksPerSec, double rippleTicksPerSec, double busVolts) {}

  /** Power setpoints to sample. Below ~0.25 a loaded flywheel may not turn at all. */
  private static final double[] POWER_STEPS = {
    0.25, 0.325, 0.40, 0.475, 0.55, 0.625, 0.70, 0.775, 0.85, 0.925, 1.0
  };

  /** Time held at each setpoint before sampling, and the length of the sampling window. */
  private static final double SETTLE_SEC = 1.6;

  private static final double SAMPLE_SEC = 0.8;

  /**
   * Proportional gain as a multiple of kV (loop gain), and the integral time: the pair
   * FlywheelClosedLoopProperties found most robust across calibration error, battery voltage and
   * velocity-sensor lag. Loop gain 0.3 with no integral left the wheel outside the feed band
   * whenever kS or kV was a few percent off; above ~3 it tolerates less lag and hides ball dips.
   */
  private static final double LOOP_GAIN_TARGET = 2.0;

  private static final double INTEGRAL_TIME_SEC = 2.0;

  private List<LynxModule> allHubs;
  private TelemetryManager telemetryM;
  private DcMotorEx shooter1;
  private DcMotorEx shooter2;

  private final List<SweepPoint> sweep = new ArrayList<>();

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT

    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
    shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

    shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    shooter1.setDirection(DcMotorSimple.Direction.FORWARD);

    shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

    stopMotors();

    telemetry.addLine("=== SHOOTER CHARACTERIZATION ===");
    telemetry.addLine("Holds a staircase of open-loop powers and records settled velocity.");
    telemetry.addLine("Fits u = kV*v + kS across ALL points, then recommends F, kS and P.");
    telemetry.addLine("");
    telemetry.addLine("Flywheel must be CLEAR of game elements. Takes about 30 seconds.");
    telemetry.addLine("Press [A] to start.");
    telemetry.update();

    waitForStart();

    while (opModeIsActive() && !isStopRequested() && !gamepad1.a) {
      clearBulkCache();
      idle();
    }

    runSweep();
    stopMotors();

    if (sweep.size() < 3) {
      reportAborted();
      return;
    }

    List<FlywheelFeedforwardFit.Sample> samples = new ArrayList<>();
    for (SweepPoint p : sweep) {
      // Shooter scales its feedforward by nominal_voltage / measured volts, so its coefficients
      // describe the wheel at nominal voltage. Fit the power that would have produced this speed
      // at nominal; fitting raw power made feedforward ~9% low after characterizing on a fresh
      // 13.2 V battery, the wheel settled under min_transfer_threshold and nothing fired.
      double atNominal = p.power() * p.busVolts() / config.shooter.nominal_voltage;
      samples.add(new FlywheelFeedforwardFit.Sample(atNominal, p.velocityTicksPerSec()));
    }
    reportResults(FlywheelFeedforwardFit.fit(samples));
  }

  /** Holds each power step, waits for settle, then averages velocity over a sampling window. */
  private void runSweep() throws InterruptedException {
    ElapsedTime timer = new ElapsedTime();

    for (double power : POWER_STEPS) {
      if (!opModeIsActive() || isStopRequested()) return;

      setPowerRaw(power);

      timer.reset();
      while (opModeIsActive() && !isStopRequested() && timer.seconds() < SETTLE_SEC) {
        clearBulkCache();
        telemetry.addLine("SETTLING");
        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Velocity", "%.1f ticks/s", Math.abs(shooter1.getVelocity()));
        telemetry.addData("Settle", "%.1f / %.1f s", timer.seconds(), SETTLE_SEC);
        telemetry.update();
        sleep(10);
      }

      double voltSum = 0.0;
      double sum = 0.0;
      double min = Double.MAX_VALUE;
      double max = -Double.MAX_VALUE;
      int n = 0;

      timer.reset();
      while (opModeIsActive() && !isStopRequested() && timer.seconds() < SAMPLE_SEC) {
        clearBulkCache();
        double v = Math.abs(shooter1.getVelocity());
        voltSum += busVoltage();
        sum += v;
        min = Math.min(min, v);
        max = Math.max(max, v);
        n++;

        telemetry.addLine("SAMPLING");
        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Velocity", "%.1f ticks/s", v);
        telemetry.addData("Running mean", "%.1f ticks/s", sum / n);
        telemetry.update();
        sleep(10);
      }

      if (n > 0) {
        double mean = sum / n;
        // Discard points where the wheel never actually broke loose.
        if (mean > 20.0) {
          sweep.add(new SweepPoint(power, mean, max - min, voltSum / n));
        }
      }
    }

    stopMotors();
  }

  private void reportAborted() {
    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      telemetry.addLine("=== CHARACTERIZATION ABORTED ===");
      telemetry.addData("Usable points", sweep.size());
      telemetry.addLine("The flywheel never spun at the sampled powers.");
      telemetry.addLine("Check motor names, wiring, and that nothing is jamming the wheel.");
      telemetry.update();
      sleep(50);
    }
  }

  private void reportResults(FlywheelFeedforwardFit.Result fit) {
    double kV = fit.kV();
    double kS = fit.kS();
    double rms = fit.rmsResidual();
    double maxResidual = fit.maxResidual();
    double fittedF = fit.f();
    double vMaxExtrapolated = fit.vMax();
    double recommendedP = fit.proportionalGainFor(LOOP_GAIN_TARGET);

    double target = config.shooter.constant_rpm;
    double predictedCommand = fit.predictPower(target);

    saveToFile(kV, kS, fittedF, recommendedP, rms, maxResidual);

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();

      telemetry.addLine("=== CHARACTERIZATION COMPLETE ===");
      telemetry.addData("Points fitted", sweep.size());
      telemetry.addData("Fit RMS error", "%.4f power", rms);
      telemetry.addData("Fit worst error", "%.4f power", maxResidual);
      telemetry.addLine("");
      telemetry.addData("kS (intercept)", "%.4f", kS);
      telemetry.addData("kV", "%.7f power per tick/s", kV);
      telemetry.addData("vMax extrapolated", "%.1f ticks/s", vMaxExtrapolated);
      telemetry.addLine("");
      telemetry.addData("Target", "%.0f ticks/s", target);
      telemetry.addData("Predicted open-loop cmd", "%.4f", predictedCommand);
      telemetry.addLine("");
      if (config.shooter.use_ftc_pid) {
        telemetry.addLine("!! shooter.use_ftc_pid is true: the robot uses motor_pidf and IGNORES");
        telemetry.addLine("!! the values below. Set use_ftc_pid: false to use them.");
      }
      telemetry.addLine("PASTE INTO config.yaml under shooter (fitted at nominal_voltage):");
      telemetry.addLine(String.format("  ks: %.4f", kS));
      telemetry.addLine("  pidf:");
      telemetry.addLine(String.format("    p: %.7f", recommendedP));
      telemetry.addLine(String.format("    i: %.7f", recommendedP / INTEGRAL_TIME_SEC));
      telemetry.addLine("    d: 0");
      telemetry.addLine(String.format("    f: %.4f", fittedF));
      telemetry.update();

      if (telemetryM != null) {
        telemetryM.addLine("=== SHOOTER CHARACTERIZATION ===");
        telemetryM.addData("Points", String.valueOf(sweep.size()));
        telemetryM.addData("kS", String.format("%.4f", kS));
        telemetryM.addData("kV", String.format("%.7f", kV));
        telemetryM.addData("F", String.format("%.4f", fittedF));
        telemetryM.addData("P", String.format("%.7f", recommendedP));
        telemetryM.addData("Fit RMS", String.format("%.4f", rms));
        telemetryM.addData("vMax", String.format("%.1f", vMaxExtrapolated));
        for (SweepPoint p : sweep) {
          telemetryM.addData(
              String.format("u=%.3f", p.power()),
              String.format(
                  "v=%.1f (ripple %.1f)", p.velocityTicksPerSec(), p.rippleTicksPerSec()));
        }
        telemetryM.update();
      }

      sleep(50);
    }
  }

  private void saveToFile(
      double kV, double kS, double fittedF, double p, double rms, double maxResidual) {
    File dir = new File("/sdcard/FIRST/teamcode/");
    if (!dir.exists()) {
      dir.mkdirs();
    }
    File logFile = new File(dir, "shooter_characterization.json");
    try (FileWriter writer = new FileWriter(logFile, false)) {
      writer.write("{\n");
      writer.write(
          String.format(
              "  \"fit\": {\"kv\": %.8f, \"ks\": %.4f, \"f\": %.4f, \"p\": %.8f, \"rms\": %.5f, \"max_residual\": %.5f},\n",
              kV, kS, fittedF, p, rms, maxResidual));
      writer.write("  \"sweep\": [\n");
      for (int i = 0; i < sweep.size(); i++) {
        SweepPoint sp = sweep.get(i);
        writer.write(
            String.format(
                "    {\"power\": %.4f, \"velocity\": %.2f, \"ripple\": %.2f}%s\n",
                sp.power(),
                sp.velocityTicksPerSec(),
                sp.rippleTicksPerSec(),
                (i == sweep.size() - 1) ? "" : ","));
      }
      writer.write("  ]\n");
      writer.write("}\n");
    } catch (IOException e) {
      telemetry.addData("Error", "Failed to write characterization file: " + e.getMessage());
    }
  }

  private void setPowerRaw(double power) {
    shooter1.setPower(power);
    shooter2.setPower(power);
  }

  private void stopMotors() {
    shooter1.setPower(0);
    shooter2.setPower(0);
  }

  /** Lowest positive bus voltage, the same reading Shooter's voltage compensation uses. */
  private double busVoltage() {
    double lowest = Double.POSITIVE_INFINITY;
    for (var sensor : hardwareMap.voltageSensor) {
      double v = sensor.getVoltage();
      if (v > 0) {
        lowest = Math.min(lowest, v);
      }
    }
    return Double.isFinite(lowest) ? lowest : config.shooter.nominal_voltage;
  }

  private void clearBulkCache() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }
  }
}
