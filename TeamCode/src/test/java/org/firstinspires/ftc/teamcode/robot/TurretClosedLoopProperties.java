package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.lifecycle.AfterProperty;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * The turret's closed-loop behaviour over the space of plausible real turrets, using the real
 * {@link Turret} code and config against {@link TurretSim}. Each property is a requirement a driver
 * would notice if it broke: the turret reaches any reachable target and stays there, it never hunts
 * or drives into its stops, it never trips the stall watchdog while free to move, it tracks a
 * turning chassis in both directions, and it never reports aimed at a CELL it cannot reach.
 */
class TurretClosedLoopProperties {
  private final double savedKv = config.turret.kv;
  private final boolean savedEnabled = config.turret.enabled;
  private final boolean savedRunaway = config.turret.runaway.enabled;

  TurretClosedLoopProperties() {
    config.turret.enabled = true;
    // Every property below asserts the turret never faults, so running them all with the runaway
    // watchdog armed checks that it never trips on a correctly wired turret.
    config.turret.runaway.enabled = true;
  }

  @AfterProperty
  void restore() {
    config.turret.kv = savedKv;
    config.turret.enabled = savedEnabled;
    config.turret.runaway.enabled = savedRunaway;
  }

  /**
   * With friction near what was calibrated into {@code ks_positive}/{@code ks_negative}, any target
   * inside travel is reached within 1 s of the fastest slew the servo allows at {@code
   * max_power_output}, and held, from anywhere in travel. The second covers the proportional final
   * approach, whose time constant is 1 / (pidf.p * k): about 0.6 s on the slowest plant.
   */
  @Property(tries = 300)
  void calibratedTurretReachesAndHoldsAnyReachableTarget(
      @ForAll("calibratedPlants") TurretSim.Plant plant,
      @ForAll("chassisPoses") Pose chassis,
      @ForAll("inTravel") double start,
      @ForAll("inTravel") double bearing,
      @ForAll @DoubleRange(min = 20, max = 140) double distance) {
    Run run = settle(plant, chassis, start, bearing, distance, 3.0);

    double stiction = bearing > start ? plant.stictionPositive() : plant.stictionNegative();
    double topSpeed = plant.k() * (config.turret.max_power_output - stiction);
    double fastest = Math.max(0, Math.abs(bearing - start) - run.toleranceDeg) / topSpeed;
    assertTrue(
        run.firstAimedSec <= fastest + 1.0,
        () -> "first aimed at " + run.firstAimedSec + " s, physics allows " + fastest + " " + run);
    assertEquals(0, run.unaimedLoopsInLastSecond, () -> "lost aim while holding " + run);
    assertTrue(run.maxOvershootDeg <= run.toleranceDeg * 1.5, () -> "overshoot " + run);
    assertTrue(run.powerReversalsInLastSecond <= 2, () -> "hunting while holding " + run);
    assertFalse(run.faulted, () -> "false fault: " + run.faultReason);
  }

  /**
   * Even if the turret's friction drifts from the calibration (wear, temperature, a rebuilt
   * gearbox), it still gets there and settles: slower is acceptable, faulting or hunting is not.
   */
  @Property(tries = 300)
  void anyPlausibleTurretStillConvergesWithoutFaultingOrHunting(
      @ForAll("anyPlants") TurretSim.Plant plant,
      @ForAll("chassisPoses") Pose chassis,
      @ForAll("inTravel") double start,
      @ForAll("inTravel") double bearing,
      @ForAll @DoubleRange(min = 20, max = 140) double distance) {
    Run run = settle(plant, chassis, start, bearing, distance, 6.0);

    assertTrue(run.firstAimedSec <= 4.0, () -> "first aimed at " + run.firstAimedSec + " s " + run);
    assertEquals(0, run.unaimedLoopsInLastSecond, () -> "lost aim while holding " + run);
    assertTrue(run.powerReversalsInLastSecond <= 2, () -> "hunting while holding " + run);
    assertFalse(run.faulted, () -> "false fault: " + run.faultReason);
  }

  /**
   * With {@code kv} calibrated, the turret keeps a fixed CELL aimed while the chassis turns either
   * way, and whenever it says it is aimed it really is: the miss at the CELL is inside the band the
   * shot gate accepts (tolerance times exit factor), allowing for one loop of chassis motion and
   * the encoder noise. The clockwise case is the one a wrapped yaw rate (a Pose heading) used to
   * break.
   */
  @Property(tries = 200)
  void tracksATurningChassisInBothDirectionsAndNeverClaimsAFalseAim(
      @ForAll("calibratedPlants") TurretSim.Plant plant,
      @ForAll("chassisPoses") Pose chassis,
      @ForAll("chassisRates") double rateDps,
      @ForAll @DoubleRange(min = 30, max = 120) double distance) {
    config.turret.kv = 1.0 / plant.k();
    // Start on the side the turn sweeps away from, so the target crosses most of the travel.
    double startBearing = Math.signum(rateDps) * (config.turret.travel.max_angle - 4);
    TurretSim sim = new TurretSim(plant, startBearing, chassis);
    sim.aimAtBearing(distance, startBearing);
    double tx = targetX(sim, distance, startBearing);
    double ty = targetY(sim, distance, startBearing);
    for (int i = 0; i < 50; i++) {
      sim.step();
    }

    double band =
        sim.turret.calculateDynamicTolerance(distance)
            * Math.max(1.0, config.turret.tolerance.exit_factor);
    double slack = Math.abs(rateDps) * 1.5 * TurretSim.LOOP_SEC + plant.encoderNoiseDeg() + 0.05;
    double sweepSec = 2 * (config.turret.travel.max_angle - 6) / Math.abs(rateDps);
    int loops = (int) (sweepSec / TurretSim.LOOP_SEC);
    int counted = 0;
    int aimedLoops = 0;
    for (int i = 0; i < loops; i++) {
      sim.step(rateDps, 0, 0);
      assertFalse(sim.turret.isFaulted(), sim.turret.getFaultReason());
      double error = Math.abs(sim.desiredAngle(tx, ty) - sim.angle);
      if (sim.turret.isAimed(sim.pose)) {
        aimedLoops++;
        assertTrue(
            error <= band + slack,
            "claimed aimed " + error + " deg off (band " + band + ") turning at " + rateDps);
      }
      counted++;
    }
    double aimedFraction = (double) aimedLoops / counted;
    assertTrue(
        aimedFraction >= 0.95,
        () -> "aimed only " + aimedFraction + " of the time turning at " + rateDps + " deg/s");
    assertFalse(sim.turret.isFaulted(), () -> sim.turret.getFaultReason());
  }

  /**
   * A CELL beyond the travel is never "aimed", and the turret waits at the stop without pushing.
   */
  @Property(tries = 200)
  void unreachableTargetsAreNeverAimedAndNeverPushTheStop(
      @ForAll("anyPlants") TurretSim.Plant plant,
      @ForAll("chassisPoses") Pose chassis,
      @ForAll("inTravel") double start,
      @ForAll("outsideTravel") double bearing,
      @ForAll @DoubleRange(min = 20, max = 140) double distance) {
    TurretSim sim = new TurretSim(plant, start, chassis);
    sim.aimAtBearing(distance, bearing);
    for (int i = 0; i < 200; i++) {
      sim.step();
      assertFalse(sim.turret.isFaulted(), sim.turret.getFaultReason());
      assertFalse(sim.turret.isAimed(sim.pose), "aimed at an unreachable CELL");
      assertTrue(sim.angle <= sim.stopMax && sim.angle >= sim.stopMin);
    }
    var travel = config.turret.travel;
    double limit = bearing > 0 ? travel.max_angle : travel.min_angle;
    // It settles within its own aim tolerance of the clamped target, i.e. the travel limit.
    double tol = sim.turret.calculateDynamicTolerance(distance) + 0.5;
    assertTrue(Math.abs(sim.angle - limit) < tol, () -> "parked at " + sim.angle);
    assertEquals(0.0, sim.power, 0.0, "must not keep driving into the stop");
    assertFalse(sim.turret.isFaulted(), () -> sim.turret.getFaultReason());
  }

  /**
   * With {@code kv} at 0, as the config ships until the plant gain is measured, a turning chassis
   * still never gets a false "aimed": the hold band may be lost while the turret catches up, but
   * whenever the shot gate is told aimed, the miss is inside the band.
   */
  @Property(tries = 200)
  void withoutKvTheTurretNeverClaimsAFalseAimWhileTheChassisTurns(
      @ForAll("calibratedPlants") TurretSim.Plant plant,
      @ForAll("chassisPoses") Pose chassis,
      @ForAll("chassisRates") double rateDps,
      @ForAll @DoubleRange(min = 30, max = 120) double distance) {
    config.turret.kv = 0.0;
    double startBearing = Math.signum(rateDps) * (config.turret.travel.max_angle - 4);
    TurretSim sim = new TurretSim(plant, startBearing, chassis);
    sim.aimAtBearing(distance, startBearing);
    double tx = targetX(sim, distance, startBearing);
    double ty = targetY(sim, distance, startBearing);
    for (int i = 0; i < 50; i++) {
      sim.step();
    }
    double band =
        sim.turret.calculateDynamicTolerance(distance)
            * Math.max(1.0, config.turret.tolerance.exit_factor);
    double slack = Math.abs(rateDps) * 1.5 * TurretSim.LOOP_SEC + plant.encoderNoiseDeg() + 0.05;
    int loops =
        (int) (2 * (config.turret.travel.max_angle - 6) / Math.abs(rateDps) / TurretSim.LOOP_SEC);
    for (int i = 0; i < loops; i++) {
      sim.step(rateDps, 0, 0);
      assertFalse(sim.turret.isFaulted(), sim.turret.getFaultReason());
      double error = Math.abs(sim.desiredAngle(tx, ty) - sim.angle);
      if (sim.turret.isAimed(sim.pose)) {
        assertTrue(error <= band + slack, "claimed aimed " + error + " deg off, band " + band);
      }
    }
    assertFalse(sim.turret.isFaulted(), () -> sim.turret.getFaultReason());
  }

  /**
   * MANUAL power is cut at each travel limit in the direction of that limit (so the turret is never
   * driven into its stop), reversing off a limit always works, and holding power against a limit is
   * not a stall.
   */
  @Property(tries = 300)
  void manualPowerStopsAtTheTravelLimitsAndReversesOffThem(
      @ForAll("anyPlants") TurretSim.Plant plant,
      @ForAll("chassisPoses") Pose chassis,
      @ForAll("inTravel") double start,
      @ForAll("manualPowers") double manualPower) {
    var travel = config.turret.travel;
    TurretSim sim = new TurretSim(plant, start, chassis);
    sim.turret.setAimMode(Turret.AimMode.MANUAL);
    sim.turret.setManualPower(manualPower);
    // Power is cut at the limit, but the servo coasts for one loop plus its own lag.
    double overrun =
        plant.k() * Math.abs(manualPower) * 1.5 * (TurretSim.LOOP_SEC + plant.lagSec()) + 0.2;
    double stiction = manualPower > 0 ? plant.stictionPositive() : plant.stictionNegative();
    double speedDps = plant.k() * (Math.abs(manualPower) - stiction);
    double travelDeg = travel.max_angle - travel.min_angle;
    int loops = (int) Math.ceil(travelDeg / speedDps / TurretSim.LOOP_SEC) + 50;
    for (int i = 0; i < loops; i++) {
      sim.step();
      assertFalse(sim.turret.isFaulted(), sim.turret.getFaultReason());
      assertTrue(sim.angle <= travel.max_angle + overrun, "drove past max: " + sim.angle);
      assertTrue(sim.angle >= travel.min_angle - overrun, "drove past min: " + sim.angle);
    }
    double limit = manualPower > 0 ? travel.max_angle : travel.min_angle;
    assertTrue(
        Math.abs(sim.angle - limit) <= overrun, () -> "did not reach the stop: " + sim.angle);
    assertEquals(0.0, sim.power, 0.0, "still pushing into the stop");
    assertFalse(sim.turret.isFaulted(), () -> sim.turret.getFaultReason());

    sim.turret.setManualPower(-manualPower);
    sim.step();
    assertEquals(-manualPower, sim.power, 1e-12, "reversing off the stop was blocked");
  }

  /**
   * isAimed() is exactly the geometry: from any field pose, a turret within the aim tolerance of
   * the bearing to the CELL (measured from the turret axis) is aimed, and one outside it is not.
   */
  @Property(tries = 1000)
  void isAimedMatchesTheGeometryFromAnywhere(
      @ForAll("chassisPoses") Pose chassis,
      @ForAll("inTravel") double bearing,
      @ForAll @DoubleRange(min = 20, max = 140) double distance,
      @ForAll @DoubleRange(min = -2, max = 2) double offsetInTolerances,
      @ForAll @DoubleRange(min = -6, max = 6) double pivotForward,
      @ForAll @DoubleRange(min = -6, max = 6) double pivotLeft) {
    double savedForward = config.turret.pivot_forward_in;
    double savedLeft = config.turret.pivot_left_in;
    config.turret.pivot_forward_in = pivotForward;
    config.turret.pivot_left_in = pivotLeft;
    try {
      TurretSim.Plant still = new TurretSim.Plant(100, 0, 0, 0, 0, 0, 0);
      TurretSim sim = new TurretSim(still, 0, chassis);
      // Target placed relative to the turret axis, the way the shot solver measures it.
      Pose pivot = Turret.pivotPose(sim.pose);
      double b = sim.pose.heading() + Math.toRadians(bearing);
      double tx = pivot.x() + distance * Math.cos(b);
      double ty = pivot.y() + distance * Math.sin(b);
      sim.turret.setTargetPoint(tx, ty);
      sim.turret.setAimMode(Turret.AimMode.AIM_AT_TARGET);

      double tol = sim.turret.calculateDynamicTolerance(distance);
      sim.angle = bearing + offsetInTolerances * tol;
      double margin = Math.abs(Math.abs(offsetInTolerances) - 1.0);
      if (margin > 0.02) {
        assertEquals(
            Math.abs(offsetInTolerances) < 1.0,
            sim.turret.isAimed(sim.pose),
            "offset "
                + offsetInTolerances
                + " tolerances ("
                + tol
                + " deg) at "
                + distance
                + " in");
      }
    } finally {
      config.turret.pivot_forward_in = savedForward;
      config.turret.pivot_left_in = savedLeft;
    }
  }

  // ---------------------------------------------------------------------------------------------

  /** What happened over one settle run. */
  record Run(
      double firstAimedSec,
      int unaimedLoopsInLastSecond,
      int powerReversalsInLastSecond,
      double maxOvershootDeg,
      double toleranceDeg,
      boolean faulted,
      String faultReason,
      double finalAngle,
      double target) {}

  private static Run settle(
      TurretSim.Plant plant,
      Pose chassis,
      double start,
      double bearing,
      double distance,
      double seconds) {
    TurretSim sim = new TurretSim(plant, start, chassis);
    sim.aimAtBearing(distance, bearing);
    int loops = (int) (seconds / TurretSim.LOOP_SEC);
    int lastSecond = loops - (int) (1.0 / TurretSim.LOOP_SEC);
    double firstAimed = Double.POSITIVE_INFINITY;
    int unaimed = 0;
    int reversals = 0;
    double lastPowerSign = 0;
    double overshoot = 0;
    double direction = Math.signum(bearing - start);
    double elapsed = 0;
    String fault = null;
    for (int i = 0; i < loops; i++) {
      sim.step();
      elapsed += TurretSim.LOOP_SEC;
      // Stall faults clear themselves after retry_after_sec, so check every loop, not just the end.
      if (fault == null && sim.turret.isFaulted()) {
        fault = sim.turret.getFaultReason();
      }
      boolean aimed = sim.turret.isAimed(sim.pose);
      if (aimed && firstAimed == Double.POSITIVE_INFINITY) {
        firstAimed = elapsed;
      }
      overshoot = Math.max(overshoot, direction * (sim.angle - bearing));
      if (i >= lastSecond) {
        if (!aimed) {
          unaimed++;
        }
        double sign = Math.signum(sim.power);
        if (sign != 0 && lastPowerSign != 0 && sign != lastPowerSign) {
          reversals++;
        }
        if (sign != 0) {
          lastPowerSign = sign;
        }
      }
    }
    return new Run(
        firstAimed,
        unaimed,
        reversals,
        overshoot,
        sim.turret.calculateDynamicTolerance(distance),
        fault != null,
        fault,
        sim.angle,
        bearing);
  }

  private static double targetX(TurretSim sim, double distance, double bearingDeg) {
    return sim.pose.x() + distance * Math.cos(sim.pose.heading() + Math.toRadians(bearingDeg));
  }

  private static double targetY(TurretSim sim, double distance, double bearingDeg) {
    return sim.pose.y() + distance * Math.sin(sim.pose.heading() + Math.toRadians(bearingDeg));
  }

  /** Anywhere on the field at any heading: the aim math must not quietly assume the origin. */
  @Provide
  Arbitrary<Pose> chassisPoses() {
    Arbitrary<Double> coord =
        Arbitraries.doubles().between(10, config.field.size_in - 10).ofScale(3);
    Arbitrary<Double> heading = Arbitraries.doubles().between(-180, 180).ofScale(3);
    return Combinators.combine(coord, coord, heading)
        .as((x, y, h) -> new Pose(x, y, Math.toRadians(h)));
  }

  @Provide
  Arbitrary<Double> inTravel() {
    var t = config.turret.travel;
    return Arbitraries.doubles().between(t.min_angle + 0.5, t.max_angle - 0.5).ofScale(3);
  }

  @Provide
  Arbitrary<Double> outsideTravel() {
    var t = config.turret.travel;
    Arbitrary<Double> beyondMax = Arbitraries.doubles().between(t.max_angle + 5, 90).ofScale(3);
    Arbitrary<Double> beyondMin = Arbitraries.doubles().between(-90, t.min_angle - 5).ofScale(3);
    return Arbitraries.oneOf(beyondMax, beyondMin);
  }

  @Provide
  Arbitrary<Double> manualPowers() {
    Arbitrary<Double> up = Arbitraries.doubles().between(0.15, 0.5).ofScale(3);
    return Arbitraries.oneOf(up, up.map(v -> -v));
  }

  @Provide
  Arbitrary<Double> chassisRates() {
    Arbitrary<Double> ccw = Arbitraries.doubles().between(10, 30).ofScale(3);
    return Arbitraries.oneOf(ccw, ccw.map(r -> -r));
  }

  /** Friction within 30% (or 0.01) of the calibrated kS in each direction. */
  @Provide
  Arbitrary<TurretSim.Plant> calibratedPlants() {
    double ksPos = config.turret.ks_positive;
    double ksNeg = config.turret.ks_negative;
    return plants(
        Arbitraries.doubles()
            .between(round5(Math.max(0, ksPos * 0.7 - 0.01)), round5(ksPos * 1.3 + 0.01))
            .ofScale(5),
        Arbitraries.doubles()
            .between(round5(Math.max(0, ksNeg * 0.7 - 0.01)), round5(ksNeg * 1.3 + 0.01))
            .ofScale(5));
  }

  /** Any friction a goBILDA/Axon CRServo turret plausibly has, regardless of calibration. */
  @Provide
  Arbitrary<TurretSim.Plant> anyPlants() {
    Arbitrary<Double> stiction = Arbitraries.doubles().between(0, 0.08).ofScale(5);
    return plants(stiction, stiction);
  }

  private static double round5(double v) {
    return Math.round(v * 1e5) / 1e5;
  }

  private static Arbitrary<TurretSim.Plant> plants(
      Arbitrary<Double> stictionPositive, Arbitrary<Double> stictionNegative) {
    return Combinators.combine(
            Arbitraries.doubles().between(80, 200).ofScale(3),
            stictionPositive,
            stictionNegative,
            Arbitraries.doubles().between(0, 0.04).ofScale(4),
            Arbitraries.doubles().between(0, 0.15).ofScale(4),
            Arbitraries.doubles().between(0, 0.006).ofScale(5),
            Arbitraries.longs())
        .as(TurretSim.Plant::new);
  }
}
