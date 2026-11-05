package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.mockito.Mockito;

/**
 * A simulated turret drivetrain wrapped around the real {@link Turret} code and the real config.
 *
 * <p>The plant is a CRServo ({@link TurretPlant}, the same physics the convergence proof assumes):
 * commanded power sets angular speed ({@code k} deg/s per unit power) once it exceeds a
 * direction-dependent static friction, through a first-order servo lag. The absolute analog encoder
 * reports the angle with noise, through the configured zero voltage, inversion and
 * volts-per-degree. Mechanical stops sit where the configured encoder voltage limits say they are.
 * The chassis can turn and translate; the loop period jitters like a real Control Hub loop.
 */
final class TurretSim {
  /** Plant parameters. Stiction is in units of servo power. */
  record Plant(
      double k,
      double stictionPositive,
      double stictionNegative,
      double lagSec,
      double encoderNoiseDeg,
      double loopJitterSec,
      long seed) {}

  static final double LOOP_SEC = 0.020;

  final Plant plant;
  final Turret turret;
  final double stopMin;
  final double stopMax;
  private final Random random;

  double angle;
  double power;
  double servoOutput;
  boolean jammed;
  long nanos = 1_000_000_000L;
  Pose pose;
  Velocity velocity = Velocity.zero();

  TurretSim(Plant plant, double startAngle, Pose chassis) {
    this.plant = plant;
    this.pose = chassis;
    this.random = new Random(plant.seed());
    var enc = config.turret.analog_encoder;
    double a = angleAtVoltage(enc.min_voltage);
    double b = angleAtVoltage(enc.max_voltage);
    stopMin = Math.min(a, b);
    stopMax = Math.max(a, b);
    angle = Math.clamp(startAngle, stopMin, stopMax);

    // Plain fakes for the two devices the loop reads and writes every step: a Mockito invocation
    // records a stack trace each call, which made the simulation 5x slower than the code under
    // test.
    HardwareMap hw = Mockito.mock(HardwareMap.class, Mockito.withSettings().stubOnly());
    CRServo servo =
        new CRServoImpl(null, 0) {
          @Override
          public void setPower(double p) {
            power = p;
          }

          @Override
          public double getPower() {
            return power;
          }
        };
    AnalogInput analog =
        new AnalogInput(null, 0) {
          @Override
          public double getVoltage() {
            return voltageFor(measuredAngle());
          }
        };
    Mockito.when(hw.get(CRServo.class, "turn")).thenReturn(servo);
    Mockito.when(hw.get(AnalogInput.class, "turnanalog")).thenReturn(analog);
    turret = new Turret(hw, (Telemetry) null, () -> pose, () -> velocity, () -> nanos);
  }

  private double measuredAngle() {
    return angle + plant.encoderNoiseDeg() * (2 * random.nextDouble() - 1);
  }

  /** Encoder voltage for a turret angle, the inverse of the configured transfer function. */
  static double voltageFor(double angleDeg) {
    var enc = config.turret.analog_encoder;
    double v = enc.zero_voltage + (enc.inverted ? -angleDeg : angleDeg) / enc.degrees_per_volt;
    if (enc.full_scale_voltage > 0) {
      v = ((v % enc.full_scale_voltage) + enc.full_scale_voltage) % enc.full_scale_voltage;
    }
    return v;
  }

  private static double angleAtVoltage(double v) {
    var enc = config.turret.analog_encoder;
    double deg = (v - enc.zero_voltage) * enc.degrees_per_volt;
    return enc.inverted ? -deg : deg;
  }

  /** One loop: the turret decides and writes, then the plant and chassis advance. */
  void step(double chassisRateDps, double chassisVx, double chassisVy) {
    double dt = LOOP_SEC + plant.loopJitterSec() * (2 * random.nextDouble() - 1);
    nanos += (long) (dt * 1e9);
    turret.periodic();

    servoOutput = TurretPlant.servoOutput(servoOutput, power, dt, plant.lagSec());
    double effective =
        TurretPlant.effective(servoOutput, plant.stictionPositive(), plant.stictionNegative());
    if (!jammed) {
      angle = TurretPlant.angle(angle, effective, plant.k(), dt, stopMin, stopMax);
    }

    double omega = Math.toRadians(chassisRateDps);
    velocity = new Velocity(chassisVx, chassisVy, omega);
    pose =
        new Pose(pose.x() + chassisVx * dt, pose.y() + chassisVy * dt, pose.heading() + omega * dt);
  }

  void step() {
    step(0, 0, 0);
  }

  /** Aim at a field point {@code distance} in away at {@code bearingDeg} from the current pose. */
  void aimAtBearing(double distance, double bearingDeg) {
    double b = pose.heading() + Math.toRadians(bearingDeg);
    turret.setTargetPoint(pose.x() + distance * Math.cos(b), pose.y() + distance * Math.sin(b));
    turret.setAimMode(Turret.AimMode.AIM_AT_TARGET);
  }

  /** Where the turret should point right now, chassis-relative degrees, before travel clamping. */
  double desiredAngle(double targetX, double targetY) {
    Pose pivot = Turret.pivotPose(pose);
    double bearing = Math.toDegrees(Math.atan2(targetY - pivot.y(), targetX - pivot.x()));
    double rel = bearing - Math.toDegrees(pose.heading());
    return ((rel + 540) % 360) - 180;
  }
}
