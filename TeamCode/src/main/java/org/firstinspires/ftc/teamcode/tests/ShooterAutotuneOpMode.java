package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Retired: use Shooter Characterization. Its p = 0.3 kV reproduces the current gains exactly, while
 * this search steps kP by 13x the current value, accepts kP = 0 whenever feedforward alone reaches
 * the target, and on failure falls back to a gain ~300x the current one that oscillates hard.
 */
@Disabled
@TeleOp(name = "Shooter Autotune", group = "Calibration")
@Configurable
public class ShooterAutotuneOpMode extends LinearOpMode {

  private List<LynxModule> allHubs;
  private TelemetryManager telemetryM;

  private DcMotorEx shooter1;
  private DcMotorEx shooter2;

  private double tunedKs = 0.0;
  private double tunedF = 22.8;
  private double tunedKv = 0.0;
  private double tunedKp = 0.0;
  private double maxObservedVmax = 0.0;

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

    telemetry.addLine("=== SHOOTER AUTOMATED AUTOTUNER ===");
    telemetry.addLine("Calibrates static friction (kS), velocity scale (kV/F), and kP.");
    telemetry.addLine("Make sure flywheel is clear of any game elements!");
    telemetry.addLine("---------------------------------------------------------");
    telemetry.addLine("Press [A] to START Automated 3-Phase Autotune");
    telemetry.update();

    if (telemetryM != null) {
      telemetryM.addLine("=== SHOOTER AUTOMATED AUTOTUNER ===");
      telemetryM.addLine("Press [A] to START");
      telemetryM.update();
    }

    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();

      if (gamepad1.a) {
        runAutotuneSequence();
        break;
      }

      idle();
    }

    stopMotors();
  }

  private void runAutotuneSequence() throws InterruptedException {
    // --- PHASE 1: STATIC FRICTION (kS) STICTION RAMP ---
    telemetry.addLine("Phase 1: Measuring Static Friction (kS)...");
    telemetry.update();
    sleep(500);

    double powerRamp = 0.0;
    int consecutiveMovingFrames = 0;
    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      powerRamp += 0.0004;
      setPowerRaw(powerRamp);

      double currentVel = Math.abs(shooter1.getVelocity());
      if (currentVel > 10.0) {
        consecutiveMovingFrames++;
      } else {
        consecutiveMovingFrames = 0;
      }

      telemetry.addLine("PHASE 1: STICTION RAMP");
      telemetry.addData("Power", "%.4f", powerRamp);
      telemetry.addData("Velocity", "%.1f ticks/s", currentVel);
      telemetry.update();

      if (consecutiveMovingFrames >= 5) {
        tunedKs = powerRamp;
        break;
      }
      sleep(10);
    }

    stopMotors();
    sleep(1000);

    // --- PHASE 2: MAX VELOCITY STEP (kV / F) ---
    telemetry.addLine("Phase 2: Measuring Max Velocity (kV / F)...");
    telemetry.update();

    ElapsedTime timer = new ElapsedTime();
    timer.reset();
    maxObservedVmax = 0.0;

    while (opModeIsActive() && !isStopRequested() && timer.seconds() < 4.0) {
      clearBulkCache();
      setPowerRaw(1.0);
      double currentVel = Math.abs(shooter1.getVelocity());
      if (timer.seconds() > 1.2 && currentVel > maxObservedVmax) {
        maxObservedVmax = currentVel;
      }

      telemetry.addLine("PHASE 2: MAX VELOCITY STEP");
      telemetry.addData("Time", "%.1f / 4.0 s", timer.seconds());
      telemetry.addData("Velocity", "%.1f ticks/s", currentVel);
      telemetry.addData("Max Observed Vmax", "%.1f ticks/s", maxObservedVmax);
      telemetry.update();

      sleep(10);
    }

    stopMotors();
    sleep(1000);

    if (maxObservedVmax > 100.0) {
      tunedF = ((1.0 - tunedKs) * 32767.0) / maxObservedVmax;
      tunedKv = (1.0 - tunedKs) / maxObservedVmax;
    }

    // --- PHASE 3: KP STEP-RESPONSE SETTLING ---
    telemetry.addLine("Phase 3: Tuning Proportional Gain (kP)...");
    telemetry.update();

    double targetRPM = config.shooter.constant_rpm;
    double maxThreshold = targetRPM * config.shooter.max_velocity_threshold;
    double testKp = 0.0;
    boolean kpFound = false;

    for (testKp = 0.0; testKp <= 0.05; testKp += 0.002) {
      if (!opModeIsActive() || isStopRequested()) break;

      stopMotors();
      sleep(500);

      timer.reset();
      boolean reachedSetpoint = false;
      boolean overshot = false;

      while (opModeIsActive() && !isStopRequested() && timer.seconds() < 2.5) {
        clearBulkCache();

        double vel = Math.abs(shooter1.getVelocity());
        double error = targetRPM - vel;
        double ff = (tunedF * targetRPM) / 32767.0;
        double cmd = (testKp * error) + ff + Math.copySign(tunedKs, targetRPM);
        setPowerRaw(Math.clamp(cmd, -1.0, 1.0));

        if (vel >= targetRPM && timer.seconds() <= 1.8) {
          reachedSetpoint = true;
        }
        if (vel > maxThreshold) {
          overshot = true;
        }

        telemetry.addLine("PHASE 3: KP STEP RESPONSE");
        telemetry.addData("Testing kP", "%.4f", testKp);
        telemetry.addData("Target Velocity", "%.1f", targetRPM);
        telemetry.addData("Current Velocity", "%.1f", vel);
        telemetry.addData("Max Allowed Limit", "%.1f", maxThreshold);
        telemetry.update();

        sleep(10);
      }

      if (reachedSetpoint && !overshot) {
        tunedKp = testKp;
        kpFound = true;
        break;
      }
    }

    if (!kpFound) {
      tunedKp = Math.max(0.0, testKp - 0.004);
    }

    stopMotors();

    // --- RESULTS ---
    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();

      telemetry.addLine("=== AUTOTUNE COMPLETE ===");
      telemetry.addData("Max Observed Vmax", "%.1f ticks/s", maxObservedVmax);
      telemetry.addData("Tuned kS", "%.4f", tunedKs);
      telemetry.addData("Tuned F (SDK)", "%.4f", tunedF);
      telemetry.addData("Tuned kP (Pedro)", "%.6f", tunedKp);
      telemetry.addLine("---------------------------------------------");
      telemetry.addLine("PASTE INTO config.yaml under shooter:");
      telemetry.addLine(String.format("  ks: %.4f", tunedKs));
      telemetry.addLine("  pidf:");
      telemetry.addLine(String.format("    p: %.6f", tunedKp));
      telemetry.addLine(String.format("    f: %.4f", tunedF));
      telemetry.update();

      if (telemetryM != null) {
        telemetryM.addLine("=== SHOOTER AUTOTUNE COMPLETE ===");
        telemetryM.addData("Max Vmax (ticks/s)", String.format("%.1f", maxObservedVmax));
        telemetryM.addData("Tuned kS", String.format("%.4f", tunedKs));
        telemetryM.addData("Tuned F", String.format("%.4f", tunedF));
        telemetryM.addData("Tuned kP", String.format("%.6f", tunedKp));
        telemetryM.update();
      }

      sleep(50);
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

  private void clearBulkCache() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }
  }
}
