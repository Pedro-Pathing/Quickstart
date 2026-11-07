package org.firstinspires.ftc.teamcode.robot;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import static org.mockito.Mockito.withSettings;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * A simulated double flywheel wrapped around the real {@link Shooter} code and the real config.
 *
 * <p>The plant is a DC motor with a first-order mechanical lag: at power u on a battery of V volts
 * the wheel heads for (u V / V_nominal - kS) / kV ticks/s with time constant {@code tau}, the true
 * kS and kV differing from the configured ones by a calibration error. A launched ball costs a
 * fraction of the wheel's speed at once (12-26% on this robot, per the Shot Timing Tuner's traces).
 * The encoder reads with noise and lags by a loop or two (the hub derives velocity over a window),
 * and the loop period jitters like a Control Hub loop.
 */
final class FlywheelSim {
  /** True plant. kS in nominal-voltage power, kV in nominal-voltage power per tick/s. */
  record Plant(
      double ks,
      double kv,
      double tauSec,
      double busVolts,
      double encoderNoise,
      double loopJitterSec,
      int measurementLagLoops,
      long seed) {}

  static final double LOOP_SEC = 0.020;

  final Plant plant;
  final Shooter shooter;
  private final Random random;
  private final AtomicReference<Double> power = new AtomicReference<>(0.0);
  double omega;
  private final java.util.ArrayDeque<Double> history = new java.util.ArrayDeque<>();
  long nanos = 1_000_000_000L;

  FlywheelSim(Plant plant) {
    this.plant = plant;
    this.random = new Random(plant.seed());
    HardwareMap map = mock(HardwareMap.class, withSettings().stubOnly());
    DcMotorEx first = mock(DcMotorEx.class, withSettings().stubOnly());
    DcMotorEx second = mock(DcMotorEx.class, withSettings().stubOnly());
    when(map.get(DcMotorEx.class, "shooter")).thenReturn(first);
    when(map.get(DcMotorEx.class, "shooter2")).thenReturn(second);
    when(map.get(Servo.class, "hood")).thenReturn(mock(Servo.class, withSettings().stubOnly()));
    @SuppressWarnings("unchecked")
    HardwareMap.DeviceMapping<VoltageSensor> sensors =
        mock(HardwareMap.DeviceMapping.class, withSettings().stubOnly());
    map.voltageSensor = sensors;
    VoltageSensor voltage = mock(VoltageSensor.class, withSettings().stubOnly());
    when(voltage.getVoltage()).thenReturn(plant.busVolts());
    when(sensors.iterator()).thenAnswer(inv -> List.of(voltage).iterator());
    for (DcMotorEx motor : new DcMotorEx[] {first, second}) {
      AtomicReference<DcMotor.RunMode> mode = new AtomicReference<>();
      doAnswer(
              inv -> {
                mode.set(inv.getArgument(0));
                return null;
              })
          .when(motor)
          .setMode(any());
      when(motor.getMode()).thenAnswer(inv -> mode.get());
      when(motor.getVelocity()).thenAnswer(inv -> measured());
    }
    doAnswer(
            inv -> {
              power.set(inv.getArgument(0));
              return null;
            })
        .when(first)
        .setPower(anyDouble());
    shooter = new Shooter(map, () -> nanos);
  }

  private double measured() {
    double lagged =
        plant.measurementLagLoops() == 0 || history.isEmpty() ? omega : history.peekFirst();
    return lagged + plant.encoderNoise() * (2 * random.nextDouble() - 1);
  }

  double power() {
    return power.get();
  }

  /** One loop: the shooter decides and writes, then the wheel advances. */
  void step() {
    double dt = LOOP_SEC + plant.loopJitterSec() * (2 * random.nextDouble() - 1);
    nanos += (long) (dt * 1e9);
    shooter.periodic();
    double u = power.get() * plant.busVolts() / config.shooter.nominal_voltage;
    double target =
        Math.abs(u) <= plant.ks() ? 0.0 : Math.signum(u) * (Math.abs(u) - plant.ks()) / plant.kv();
    omega += (target - omega) * Math.min(1.0, dt / plant.tauSec());
    history.addLast(omega);
    // Keep lag + 1 speeds: the oldest is the one the encoder reports, lag loops behind.
    while (history.size() > plant.measurementLagLoops() + 1) {
      history.removeFirst();
    }
  }

  /** A ball leaves the wheel: it costs {@code fraction} of the current speed at once. */
  void launch(double fraction) {
    omega *= 1.0 - fraction;
  }
}
