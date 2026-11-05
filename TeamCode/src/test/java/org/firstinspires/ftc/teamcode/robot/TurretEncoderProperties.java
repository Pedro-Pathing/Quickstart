package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;
import net.jqwik.api.Assume;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.lifecycle.AfterProperty;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * The absolute analog encoder's transfer function. The sensor's output wraps at {@code
 * full_scale_voltage}; {@code zero_voltage} decides where in the turret's travel that wrap lands.
 * Re-zeroing the turret moves it, so these hold for any zero voltage, in either direction.
 */
class TurretEncoderProperties {
  private final double savedZero = config.turret.analog_encoder.zero_voltage;
  private final boolean savedInverted = config.turret.analog_encoder.inverted;

  @AfterProperty
  void restore() {
    config.turret.analog_encoder.zero_voltage = savedZero;
    config.turret.analog_encoder.inverted = savedInverted;
  }

  private static Turret turretWith(double zeroVoltage, boolean inverted) {
    config.turret.analog_encoder.zero_voltage = zeroVoltage;
    config.turret.analog_encoder.inverted = inverted;
    return new TurretSim(new TurretSim.Plant(100, 0, 0, 0, 0, 0, 0), 0, new Pose(72, 72, 0)).turret;
  }

  /** Any angle within half a sensor revolution of zero reads back exactly, across the wrap. */
  @Property(tries = 2000)
  void everyAngleWithinHalfARevolutionReadsBack(
      @ForAll @DoubleRange(min = 0, max = 3.3) double zeroVoltage,
      @ForAll boolean inverted,
      @ForAll @DoubleRange(min = -1, max = 1) double fractionOfHalfRevolution) {
    var enc = config.turret.analog_encoder;
    Turret turret = turretWith(zeroVoltage, inverted);
    double halfRevolutionDeg = enc.full_scale_voltage / 2.0 * enc.degrees_per_volt;
    double angle = 0.999 * fractionOfHalfRevolution * halfRevolutionDeg;

    assertEquals(angle, turret.angleForVoltage(TurretSim.voltageFor(angle)), 1e-9);
  }

  /**
   * The INIT warning about the wrap fires exactly when the zero voltage puts the sensor's wrap
   * between the turret's two mechanical stops, where the reading would jump mid-travel.
   */
  @Property(tries = 2000)
  void theWrapWarningFiresExactlyWhenTheWrapIsInsideTheTravel(
      @ForAll @DoubleRange(min = 0, max = 3.3) double zeroVoltage, @ForAll boolean inverted) {
    var enc = config.turret.analog_encoder;
    double fullScale = enc.full_scale_voltage;
    double wrapVoltage = ((zeroVoltage + fullScale / 2.0) % fullScale + fullScale) % fullScale;
    // Stay clear of the stops themselves, where 1 deg of slack in the check decides.
    double clearance = 1.5 / enc.degrees_per_volt;
    Assume.that(
        Math.abs(wrapVoltage - enc.min_voltage) > clearance
            && Math.abs(wrapVoltage - enc.max_voltage) > clearance);
    boolean wrapInside = wrapVoltage > enc.min_voltage && wrapVoltage < enc.max_voltage;

    String warning = turretWith(zeroVoltage, inverted).getEncoderRangeWarning();

    assertEquals(
        wrapInside,
        warning != null && warning.contains("wrap"),
        "zero " + zeroVoltage + " V, wrap at " + wrapVoltage + " V: " + warning);
  }

  @Property(tries = 1)
  void theShippedCalibrationHasNoWarning() {
    var enc = config.turret.analog_encoder;
    assertEquals(null, turretWith(savedZero, savedInverted).getEncoderRangeWarning());
    assertEquals(0.0, turretWith(savedZero, savedInverted).angleForVoltage(enc.zero_voltage), 0.0);
  }
}
