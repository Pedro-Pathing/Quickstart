package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Assume;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.constraints.IntRange;
import net.jqwik.api.lifecycle.AfterProperty;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;

/**
 * The flywheel's closed loop (feedforward kS + kV at nominal voltage, voltage compensation, P, and
 * the clamped integrator) over the space of plausible real flywheels, using the real {@link
 * Shooter} code and config against {@link FlywheelSim}. The plant's true kS and kV are off from the
 * configured ones by a calibration error (kS +/-30%, kV +/-10%), because they will be:
 * characterization runs on one battery, matches are played on others, and wheels wear. Requirements
 * a driver would notice: the wheel reaches the feed band and stays in it, recovers after every ball
 * without ringing, and the ball counter sees each ball exactly once.
 */
class FlywheelClosedLoopProperties {
  private final boolean savedFtcPid = config.shooter.use_ftc_pid;

  FlywheelClosedLoopProperties() {
    config.shooter.use_ftc_pid = false;
  }

  @AfterProperty
  void restore() {
    config.shooter.use_ftc_pid = savedFtcPid;
  }

  /** Feed band: the speed ratio the shot gate accepts. */
  private static boolean inBand(double omega, double target) {
    double ratio = omega / target;
    return ratio >= config.shooter.min_transfer_threshold
        && ratio <= config.shooter.max_velocity_threshold;
  }

  /** Target speeds (ticks/s) the solver can ask for, and the plant can physically reach. */
  private static double target(FlywheelSim.Plant plant, double fraction) {
    var b = config.shooter.ballistics;
    double rpm = b.preferred_shot_rpm + fraction * (b.max_shot_rpm - b.preferred_shot_rpm);
    double needed = plant.ks() + plant.kv() * rpm;
    double available = plant.busVolts() / config.shooter.nominal_voltage;
    Assume.that(needed <= 0.95 * available);
    return rpm;
  }

  /**
   * Any plausible flywheel spins up into the feed band within 3 s plus four of its own time
   * constants plus three integral times (the integrator must learn the feedforward's error) and
   * then stays in it: at least 95% of loops (encoder noise may flick one sample out), with no
   * reverse power.
   */
  @Property(tries = 500)
  void reachesTheFeedBandAndStaysThere(
      @ForAll("plants") FlywheelSim.Plant plant,
      @ForAll @DoubleRange(min = 0, max = 1) double where) {
    double target = target(plant, where);
    FlywheelSim sim = new FlywheelSim(plant);
    sim.shooter.setTargetPower(target / config.shooter.max_rpm);
    double integralTimeSec = config.shooter.pidf.p / config.shooter.pidf.i;
    int loops = (int) ((4.0 + 4 * plant.tauSec() + 3 * integralTimeSec) / FlywheelSim.LOOP_SEC);
    int lastSecond = loops - (int) (1.0 / FlywheelSim.LOOP_SEC);
    int inBand = 0;
    for (int i = 0; i < loops; i++) {
      sim.step();
      assertTrue(sim.power() >= 0.0, "reverse power " + sim.power());
      if (i >= lastSecond && inBand(sim.omega, target)) {
        inBand++;
      }
    }
    double fraction = (double) inBand / (loops - lastSecond);
    assertTrue(
        fraction >= 0.95,
        () ->
            "in the feed band " + fraction + " of the last second, ended at " + sim.omega / target);
  }

  /**
   * After a ball the wheel comes back into the band within four plant time constants plus two
   * integral times (the integrator re-settles too) and peaks no more than 2% over the band's top.
   * Ringing further would keep the gate shut and look like a ball; the 2% covers the worst corner
   * (kS and kV both far off, a fast wheel, two loops of sensor lag), which peaks at about 1.048.
   */
  @Property(tries = 500)
  void recoversFromEveryBallWithoutRinging(
      @ForAll("plants") FlywheelSim.Plant plant,
      @ForAll @DoubleRange(min = 0, max = 1) double where,
      @ForAll @DoubleRange(min = 0.12, max = 0.26) double dip) {
    double target = target(plant, where);
    FlywheelSim sim = new FlywheelSim(plant);
    sim.shooter.setTargetPower(target / config.shooter.max_rpm);
    for (int i = 0; i < 200; i++) {
      sim.step();
    }
    sim.launch(dip);
    double integralTimeSec = config.shooter.pidf.p / config.shooter.pidf.i;
    double recoverySec = 4 * plant.tauSec() + 2 * integralTimeSec + 0.5;
    int loops = (int) (recoverySec / FlywheelSim.LOOP_SEC);
    double peak = 0;
    boolean back = false;
    for (int i = 0; i < loops + 50; i++) {
      sim.step();
      peak = Math.max(peak, sim.omega / target);
      if (i == loops) {
        back = inBand(sim.omega, target);
      }
    }
    boolean recovered = back;
    double worst = peak;
    assertTrue(
        recovered,
        () -> "not back in band " + recoverySec + " s after a ball: " + sim.omega / target);
    assertTrue(
        worst <= config.shooter.max_velocity_threshold + 0.02,
        () -> "overshot to " + worst + " of target after a ball");
  }

  /**
   * The dip detector, fed by this loop, counts each launched ball exactly once, for balls at least
   * 0.3 s apart (AUTO allows auto.shoot_wait_ms = 2.5 s for a four-ball volley). Closer than the
   * wheel can rebound off a dip, two balls read as one on a heavy wheel.
   */
  @Property(tries = 300)
  void theBallCounterSeesEachBallOnce(
      @ForAll("plants") FlywheelSim.Plant plant,
      @ForAll @DoubleRange(min = 0, max = 1) double where,
      @ForAll @IntRange(min = 1, max = 4) int balls,
      @ForAll @DoubleRange(min = 0.12, max = 0.26) double dip,
      @ForAll @DoubleRange(min = 0.3, max = 0.8) double spacingSec) {
    double target = target(plant, where);
    FlywheelSim sim = new FlywheelSim(plant);
    sim.shooter.setTargetPower(target / config.shooter.max_rpm);
    for (int i = 0; i < 200; i++) {
      sim.step();
    }
    var d = config.shooter.ball_detection;
    FlywheelDipDetector detector =
        new FlywheelDipDetector(
            d.dip_fraction, d.rebound_fraction, d.baseline_alpha, d.refractory_ms);
    int spacing = (int) Math.round(spacingSec / FlywheelSim.LOOP_SEC);
    int counted = 0;
    long t = 0;
    // The detector learns the wheel's settled speed before the first ball, as it does in a match.
    for (int i = 0; i < 50; i++) {
      sim.step();
      t += (long) (FlywheelSim.LOOP_SEC * 1000);
      detector.update(sim.shooter.getShooterVelocity(), t);
    }
    for (int i = 0; i < balls * spacing + 150; i++) {
      if (i % spacing == 0 && i / spacing < balls) {
        sim.launch(dip);
      }
      // Robot.update order: ShotController.periodic() feeds the detector, then Shooter.periodic()
      // reacts. Reading after the reaction would see a dip already part-recovered.
      t += (long) (FlywheelSim.LOOP_SEC * 1000);
      if (detector.update(sim.shooter.getShooterVelocity(), t)) {
        counted++;
      }
      sim.step();
    }
    assertEquals(balls, counted, "balls counted");
  }

  @Provide
  Arbitrary<FlywheelSim.Plant> plants() {
    double ksCfg = config.shooter.ks;
    double kvCfg = config.shooter.pidf.f / 32767.0;
    return Combinators.combine(
            // Characterization error after a voltage-normalized fit (Shooter Characterization):
            // kS +/-30%, kV +/-10%. Much beyond that, P's steady error (~ that of a ball dip)
            // sits outside integral.band_ticks and the integrator cannot tell it from a ball;
            // re-characterize instead.
            Arbitraries.doubles().between(0.7, 1.3).ofScale(3).map(e -> ksCfg * e),
            Arbitraries.doubles().between(0.9, 1.1).ofScale(3).map(e -> kvCfg * e),
            Arbitraries.doubles().between(0.15, 1.0).ofScale(3),
            Arbitraries.doubles().between(11.5, 13.8).ofScale(2),
            Arbitraries.doubles().between(0, 15).ofScale(2),
            Arbitraries.doubles().between(0, 0.006).ofScale(4),
            Arbitraries.integers().between(0, 2),
            Arbitraries.longs())
        .as(FlywheelSim.Plant::new);
  }
}
