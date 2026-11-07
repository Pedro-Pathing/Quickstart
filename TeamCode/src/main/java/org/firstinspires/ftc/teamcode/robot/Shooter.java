package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.controllers.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.AntiWindupIntegrator;
import org.firstinspires.ftc.teamcode.utilities.FlywheelKick;

@Configurable
public class Shooter {

  private final DcMotorEx shooter1;
  private final DcMotorEx shooter2;
  private final Servo hood;
  private final VoltageSensor voltageSensor;
  private final PIDController pidfController;
  private final AntiWindupIntegrator integrator = new AntiWindupIntegrator();
  private final FlywheelKick kick = new FlywheelKick();

  /**
   * The integrator's last contribution: the feedforward's error, learned in normal loops and held
   * through a kick. The kick's model adds it to kS, so it predicts the wheel actually on the robot.
   */
  private double learnedCorrection = 0.0;

  /** Time source (ns). Injected so simulations can step the flywheel loop deterministically. */
  private final LongSupplier nanoClock;

  private long loopStartNanos;
  private long voltageReadNanos;

  private double targetPower = 0.0;
  private double targetHoodPosition;
  private double lastVelocity1 = Double.NaN;
  private double lastVelocity2 = Double.NaN;

  /** Saturation direction of the previous command: +1 railed high, -1 railed low, 0 in range. */
  private int lastSaturationSign = 0;

  // Diagnostics from the most recent periodic() pass, for telemetry only.
  private double lastTargetVelocity = 0.0;
  private double lastError = 0.0;
  private double lastPidTerm = 0.0;
  private double lastIntegralTerm = 0.0;
  private double lastFeedforwardTerm = 0.0;
  private double lastCommand = 0.0;
  private double lastBusVoltage = 0.0;
  private double lastVoltageScale = 1.0;

  public Shooter(HardwareMap hardwareMap) {
    this(hardwareMap, System::nanoTime);
  }

  Shooter(HardwareMap hardwareMap, LongSupplier nanoClock) {
    this.nanoClock = nanoClock;
    loopStartNanos = nanoClock.getAsLong();
    voltageReadNanos = loopStartNanos;
    shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
    shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
    hood = hardwareMap.get(Servo.class, "hood");
    voltageSensor = hardwareMap.voltageSensor.iterator().next();

    DcMotor.RunMode initialMode =
        config.shooter.use_ftc_pid
            ? DcMotor.RunMode.RUN_USING_ENCODER
            : DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter1.setMode(initialMode);
    shooter1.setDirection(DcMotorSimple.Direction.FORWARD);

    shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter2.setMode(initialMode);
    shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

    pidfController = new PIDController(config.shooter.pidf.p, 0.0, config.shooter.pidf.d);

    setShooterPIDFCoefficients();

    this.targetHoodPosition = config.shooter.ballistics.min_hood_servo_pos;
  }

  public final void setShooterPIDFCoefficients() {
    pidfController.kP = config.shooter.pidf.p;
    pidfController.kI = 0.0;
    pidfController.kD = config.shooter.pidf.d;

    if (config.shooter.use_ftc_pid) {
      shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.shooter.motor_pidf);
      shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.shooter.motor_pidf);
    }
  }

  public double getShooterVelocity() {
    return shooter1.getVelocity();
  }

  /** Requests a stop; hardware changes on the next periodic() pass. */
  public void stop() {
    setTargetPower(0.0);
    invalidateVelocityCache();
    resetControl();
  }

  private void invalidateVelocityCache() {
    lastVelocity1 = Double.NaN;
    lastVelocity2 = Double.NaN;
  }

  private void resetControl() {
    if (pidfController != null) {
      pidfController.reset();
    }
    integrator.reset();
    kick.reset();
    learnedCorrection = 0.0;
    lastSaturationSign = 0;
    loopStartNanos = nanoClock.getAsLong();
  }

  public void setTargetHoodPosition(double position) {
    double a = config.shooter.ballistics.min_hood_servo_pos;
    double b = config.shooter.ballistics.max_hood_servo_pos;
    this.targetHoodPosition = Math.clamp(position, Math.min(a, b), Math.max(a, b));
  }

  public double getTargetHoodPosition() {
    return targetHoodPosition;
  }

  public double getHoodPositionReadback() {
    return hood.getPosition();
  }

  public static double constantPower() {
    return config.shooter.max_rpm > 0 ? config.shooter.constant_rpm / config.shooter.max_rpm : 0.0;
  }

  /**
   * Flywheel target as a fraction of {@code max_rpm}, clamped to [0, 1]: a tuned rpm above max_rpm
   * would otherwise ask for a speed the wheel cannot reach, and the shot gate would wait on it
   * forever.
   *
   * <p>A new target keeps the integrator. What it has learned is the feedforward's error, which
   * hardly depends on the setpoint; clearing it on every re-solved shot speed restarted a 2 s
   * convergence each time the robot moved. It is cleared when the flywheel stops, and the band in
   * integral.band_ticks pauses it while the wheel moves between setpoints.
   */
  public void setTargetPower(double power) {
    this.targetPower = Double.isFinite(power) ? Math.clamp(power, 0.0, 1.0) : 0.0;
  }

  public double getTargetPower() {
    return targetPower;
  }

  public void periodic() {
    hood.setPosition(targetHoodPosition);
    DcMotor.RunMode mode =
        config.shooter.use_ftc_pid
            ? DcMotor.RunMode.RUN_USING_ENCODER
            : DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    if (shooter1.getMode() != mode) {
      shooter1.setMode(mode);
      lastVelocity1 = Double.NaN;
    }
    if (shooter2.getMode() != mode) {
      shooter2.setMode(mode);
      lastVelocity2 = Double.NaN;
    }
    if (targetPower <= 0) {
      shooter1.setPower(0);
      shooter2.setPower(0);
      if (config.shooter.use_ftc_pid) {
        writeVelocity(0.0);
      }
      resetControl();
      lastTargetVelocity = 0.0;
      lastCommand = 0.0;
    } else {
      double targetVel = config.shooter.max_rpm * targetPower;
      double currentVel = Math.abs(getShooterVelocity());

      if (config.shooter.use_ftc_pid) {
        writeVelocity(targetVel);

        lastTargetVelocity = targetVel;
        lastError = targetVel - currentVel;
        lastPidTerm = 0.0;
        lastIntegralTerm = 0.0;
        lastFeedforwardTerm = 0.0;
        lastCommand = targetPower;
      } else {
        // How long the previous command was applied, and what it delivered at nominal voltage.
        double dt = secondsSince(loopStartNanos);
        refreshBusVoltage();
        lastVoltageScale =
            voltageCompensationScale(
                config.shooter.nominal_voltage,
                lastBusVoltage,
                config.shooter.max_voltage_compensation);
        double nominalPerCommand =
            lastBusVoltage > 0.0 ? lastBusVoltage / config.shooter.nominal_voltage : 1.0;
        kick.record(lastCommand * nominalPerCommand, dt);

        double ks = config.shooter.ks;
        double kv = config.shooter.pidf.f / 32767.0;
        double ff = kv * targetVel;
        double error = targetVel - currentVel;
        var k = config.shooter.kick;
        // With the wheel model configured, P acts on the predicted speed now (a Smith predictor):
        // on the reading, which lags by sensor_lag_sec, P kept pushing for loops after a ball the
        // wheel had already recovered from, and overshot. The integrator still learns from the
        // reading, which is unbiased once the wheel settles.
        double predictedVel =
            kick.predictNow(
                currentVel, k.sensor_lag_sec, k.wheel_tau_sec, ks + learnedCorrection, kv);
        double pError = targetVel - predictedVel;
        // What the normal loop would deliver, with the integrator as it stands: the kick only
        // takes a loop when it asks for more.
        double normalCommand =
            Math.clamp(
                (config.shooter.pidf.p * pError
                        + learnedCorrection
                        + ff
                        + Math.copySign(ks, targetVel))
                    * lastVoltageScale,
                -1.0,
                1.0);
        double kickPower =
            kick.update(
                currentVel,
                targetVel,
                nominalPerCommand,
                normalCommand * nominalPerCommand,
                dt,
                k.wheel_tau_sec,
                k.sensor_lag_sec,
                k.kick_lag_margin_sec,
                ks + learnedCorrection,
                kv,
                k.kick_trigger_fraction,
                config.shooter.min_transfer_threshold,
                k.kick_speed_margin);
        if (!Double.isNaN(kickPower)) {
          // The integrator holds what it learned, as it does on any rail.
          double command = Math.clamp(kickPower / nominalPerCommand, -1.0, 1.0);
          loopStartNanos = nanoClock.getAsLong();
          lastSaturationSign = command >= 1.0 ? 1 : 0;
          lastTargetVelocity = targetVel;
          lastError = error;
          lastPidTerm = 0.0;
          lastIntegralTerm = 0.0;
          lastFeedforwardTerm = 0.0;
          lastCommand = command;
          shooter1.setPower(command);
          shooter2.setPower(command);
          return;
        }

        double pidOutput = pidfController.calculate(targetVel, pError);

        double integralTerm =
            integrator.update(
                error,
                dt,
                config.shooter.pidf.i,
                config.shooter.integral.band_ticks,
                config.shooter.integral.max_contribution,
                lastSaturationSign);
        loopStartNanos = nanoClock.getAsLong();

        double raw =
            (pidOutput + integralTerm + ff + Math.copySign(ks, targetVel)) * lastVoltageScale;
        double command = Math.clamp(raw, -1.0, 1.0);

        lastSaturationSign = raw > 1.0 ? 1 : (raw < -1.0 ? -1 : 0);

        lastTargetVelocity = targetVel;
        lastError = error;
        lastPidTerm = pidOutput;
        lastIntegralTerm = integralTerm;
        learnedCorrection = integralTerm;
        lastFeedforwardTerm = ff + Math.copySign(ks, targetVel);
        lastCommand = command;

        shooter1.setPower(command);
        shooter2.setPower(command);
      }
    }
  }

  /** The SDK caches power and servo positions, but every setVelocity call is a hub transaction. */
  private void writeVelocity(double velocity) {
    if (Double.compare(velocity, lastVelocity1) != 0) {
      shooter1.setVelocity(velocity);
      lastVelocity1 = velocity;
    }
    if (Double.compare(velocity, lastVelocity2) != 0) {
      shooter2.setVelocity(velocity);
      lastVelocity2 = velocity;
    }
  }

  private void refreshBusVoltage() {
    if (lastBusVoltage > 0.0
        && secondsSince(voltageReadNanos) < config.shooter.voltage_refresh_sec) {
      return;
    }
    voltageReadNanos = nanoClock.getAsLong();
    try {
      lastBusVoltage = voltageSensor.getVoltage();
    } catch (RuntimeException e) {
      lastBusVoltage = 0.0;
    }
  }

  private double secondsSince(long nanos) {
    return (nanoClock.getAsLong() - nanos) * 1e-9;
  }

  public static double voltageCompensationScale(
      double nominalVolts, double measuredVolts, double maxScale) {
    if (!Double.isFinite(nominalVolts) || nominalVolts <= 0.0) {
      return 1.0;
    }
    if (!Double.isFinite(measuredVolts) || measuredVolts <= 0.0) {
      return 1.0;
    }
    double scale = nominalVolts / measuredVolts;
    double limit = maxScale >= 1.0 ? maxScale : Double.MAX_VALUE;
    return Math.min(scale, limit);
  }

  public double getLastBusVoltage() {
    return lastBusVoltage;
  }

  public double getLastVoltageScale() {
    return lastVoltageScale;
  }

  public double getLastTargetVelocity() {
    return lastTargetVelocity;
  }

  public double getLastError() {
    return lastError;
  }

  public double getLastPidTerm() {
    return lastPidTerm;
  }

  public double getLastIntegralTerm() {
    return lastIntegralTerm;
  }

  public double getLastFeedforwardTerm() {
    return lastFeedforwardTerm;
  }

  public double getLastCommand() {
    return lastCommand;
  }

  /** True while the recovery kick holds full power (see {@link FlywheelKick}). */
  public boolean isKicking() {
    return kick.isKicking();
  }
}
