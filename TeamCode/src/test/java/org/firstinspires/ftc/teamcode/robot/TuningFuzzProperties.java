package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import static org.mockito.Mockito.withSettings;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.lifecycle.AfterTry;
import org.firstinspires.ftc.teamcode.ballistics.ShotSolver;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.game.MatchClock;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.yaml.snakeyaml.Yaml;

/**
 * Tuning fuzz: any combination of tunable values, each anywhere in the range config-docs.yaml
 * documents for it (or a plausible range when it documents none), must leave the robot code safe.
 * Tuning may make the robot slow or sloppy, but never: an exception, a NaN or out-of-range command
 * to a motor or servo, a turret driven past its travel or claiming an aim it does not have, a drive
 * command toward the HIVE inside the hard stop, or a "valid" shot with garbage numbers. A failure
 * shrinks to the smallest set of values that breaks one of these.
 */
class TuningFuzzProperties {
  /** Sections a team tunes; geometry, OpMode-only tuner settings and poses are left alone. */
  private static final List<String> SECTIONS =
      List.of("turret", "casablanca", "shooter", "hive", "inventory", "auto", "sentinel", "match");

  private static final List<String> SKIP_PREFIXES =
      List.of(
          "casablanca.heading_tuner",
          "casablanca.jump_tuner",
          "turret.analog_encoder.full_scale_voltage",
          "turret.analog_encoder.min_voltage",
          "turret.analog_encoder.max_voltage",
          "turret.analog_encoder.enabled",
          "turret.orientation",
          "sentinel.robot_width",
          "sentinel.geometry_epsilon_in",
          "sentinel.route_sample_step",
          "shooter.shot_table",
          "match.");

  /** One tunable leaf: its path in the facade and the range to draw from. */
  record Knob(String path, Class<?> type, double lo, double hi) {}

  private static final List<Knob> KNOBS = discoverKnobs();

  @AfterTry
  void reload() {
    config.reload();
    Casablanca casablancaDefaults = new Casablanca(new Sentinel(Alliance.RED));
    casablancaDefaults.reset();
  }

  @Property(tries = 400)
  void anyTuningLeavesTheRobotSafe(@ForAll("tunings") Map<String, Object> tuning, @ForAll long seed)
      throws Exception {
    apply(tuning);
    Random random = new Random(seed);
    turretScenario(random);
    casablancaScenario(random);
    shooterScenario(random);
    solverScenario(random);
    hiveAndClockScenario(random);
    sentinelScenario(random);
  }

  // ---------------------------------------------------------------------------------------------
  // Scenarios

  private static void turretScenario(Random random) {
    var travel = config.turret.travel;
    double start = travel.min_angle + random.nextDouble() * (travel.max_angle - travel.min_angle);
    TurretSim sim =
        new TurretSim(
            new TurretSim.Plant(120, 0.04, 0.04, 0.01, 0.05, 0.002, random.nextLong()),
            start,
            new Pose(72, 40, Math.toRadians(90)));
    // Where the turret really starts: inside its physical stops, wherever the zero voltage says.
    start = sim.angle;
    double bearing = -60 + 120 * random.nextDouble();
    double distance = 20 + 120 * random.nextDouble();
    double rate = -30 + 60 * random.nextDouble();
    sim.aimAtBearing(distance, bearing);
    double b = sim.pose.heading() + Math.toRadians(bearing);
    // The same point aimAtBearing gave the turret (placed from the robot centre).
    double tx = sim.pose.x() + distance * Math.cos(b);
    double ty = sim.pose.y() + distance * Math.sin(b);
    double maxPower = Math.min(1.0, config.turret.max_power_output);
    double overrun = 120 * maxPower * 1.5 * (TurretSim.LOOP_SEC + 0.01) + 0.3;
    double band =
        Math.max(
                Math.max(config.turret.tolerance.max_deg, config.turret.tolerance.min_deg),
                config.turret.tolerance.near_val)
            * Math.max(1.0, config.turret.tolerance.exit_factor);
    for (int i = 0; i < 100; i++) {
      sim.step(i < 50 ? 0 : rate, 0, 0);
      assertTrue(Double.isFinite(sim.power), "turret power " + sim.power);
      assertTrue(Math.abs(sim.power) <= maxPower + 1e-9, "turret power " + sim.power);
      if (!sim.turret.isEnabled()) {
        assertEquals(0.0, sim.power, 0.0, "disabled turret drove the servo");
        continue;
      }
      assertTrue(sim.angle <= Math.max(start, travel.max_angle) + overrun, "past max " + sim.angle);
      assertTrue(sim.angle >= Math.min(start, travel.min_angle) - overrun, "past min " + sim.angle);
      if (sim.turret.isAimed(sim.pose)) {
        double error = Math.abs(sim.desiredAngle(tx, ty) - sim.angle);
        double slack = Math.abs(i < 50 ? 0 : rate) * 1.5 * TurretSim.LOOP_SEC + 0.2;
        assertTrue(error <= band + slack, "claimed aimed " + error + " deg off");
      }
    }
  }

  private static void casablancaScenario(Random random) {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);
    var hive = sentinel.getProtectedZones().get(0);
    for (int i = 0; i < 60; i++) {
      Pose pose =
          new Pose(
              hive.getMinX() - 30 + random.nextDouble() * (hive.getWidth() + 60),
              hive.getMinY() - 30 + random.nextDouble() * (hive.getHeight() + 60),
              Math.toRadians(-180 + 360 * random.nextDouble()));
      Vector2D velocity =
          Vector2D.cartesian(-60 + 120 * random.nextDouble(), -60 + 120 * random.nextDouble());
      double[] out =
          casablanca.adjustDriveInput(
              pose,
              velocity,
              -4 + 8 * random.nextDouble(),
              -1 + 2 * random.nextDouble(),
              -1 + 2 * random.nextDouble(),
              -1 + 2 * random.nextDouble());
      // out[0..1] are field-frame components of a rotated stick (a full diagonal is ~1.41 on one
      // axis) that Pedro normalizes per wheel; the turn command goes to the wheels as is.
      for (double v : out) {
        assertTrue(Double.isFinite(v), "drive command " + v);
      }
      assertTrue(Math.abs(out[2]) <= 1.0 + 1e-9, "turn command " + out[2] + " beyond full power");
    }
  }

  @SuppressWarnings("unchecked")
  private static void shooterScenario(Random random) {
    HardwareMap map = mock(HardwareMap.class, withSettings().stubOnly());
    DcMotorEx first = mock(DcMotorEx.class, withSettings().stubOnly());
    DcMotorEx second = mock(DcMotorEx.class, withSettings().stubOnly());
    Servo hood = mock(Servo.class, withSettings().stubOnly());
    when(map.get(DcMotorEx.class, "shooter")).thenReturn(first);
    when(map.get(DcMotorEx.class, "shooter2")).thenReturn(second);
    when(map.get(Servo.class, "hood")).thenReturn(hood);
    map.voltageSensor = mock(HardwareMap.DeviceMapping.class, withSettings().stubOnly());
    VoltageSensor voltage = mock(VoltageSensor.class, withSettings().stubOnly());
    double bus = 9 + 5 * random.nextDouble();
    when(voltage.getVoltage()).thenReturn(bus);
    when(map.voltageSensor.iterator()).thenAnswer(inv -> List.of(voltage).iterator());
    List<String> problems = new ArrayList<>();
    AtomicReference<Double> wheel = new AtomicReference<>(0.0);
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
      when(motor.getVelocity()).thenAnswer(inv -> wheel.get());
      doAnswer(
              inv -> {
                double p = inv.getArgument(0);
                if (!Double.isFinite(p) || Math.abs(p) > 1.0 + 1e-9) {
                  problems.add("flywheel power " + p);
                }
                return null;
              })
          .when(motor)
          .setPower(anyDouble());
      doAnswer(
              inv -> {
                double v = inv.getArgument(0);
                if (!Double.isFinite(v) || v < 0 || v > config.shooter.max_rpm + 1e-6) {
                  problems.add(
                      "flywheel velocity " + v + " (max_rpm " + config.shooter.max_rpm + ")");
                }
                return null;
              })
          .when(motor)
          .setVelocity(anyDouble());
    }
    doAnswer(
            inv -> {
              double p = inv.getArgument(0);
              if (!Double.isFinite(p) || p < 0 || p > 1) {
                problems.add("hood position " + p);
              }
              return null;
            })
        .when(hood)
        .setPosition(anyDouble());

    Shooter shooter = new Shooter(map);
    for (int i = 0; i < 60; i++) {
      if (i % 15 == 0) {
        shooter.setTargetPower(random.nextDouble() * 1.2 - 0.1);
        shooter.setTargetHoodPosition(random.nextDouble() * 1.2 - 0.1);
      }
      wheel.set(config.shooter.max_rpm * random.nextDouble());
      shooter.periodic();
    }
    assertTrue(
        problems.isEmpty(),
        () -> String.join("; ", problems.subList(0, Math.min(3, problems.size()))));
  }

  private static void solverScenario(Random random) {
    FieldModel field = new FieldModel(Alliance.RED);
    BallisticsParameters params;
    try {
      params = BallisticsParameters.fromConfig();
    } catch (IllegalArgumentException e) {
      // Rejected outright: the solver refuses every shot. It must already be on the INIT report.
      assertTrue(
          org.firstinspires.ftc.teamcode.utilities.ConfigSanity.problems().stream()
              .anyMatch(p -> p.startsWith("shooter.ballistics")),
          "rejected ballistics config not reported at INIT: " + e.getMessage());
      return;
    }
    for (BallType type : BallType.values()) {
      ShotTable table;
      try {
        table = ShotTable.fromConfig(type);
      } catch (RuntimeException e) {
        String name = "shot table " + type;
        assertTrue(
            org.firstinspires.ftc.teamcode.utilities.ConfigSanity.problems().stream()
                .anyMatch(p -> p.startsWith(name)),
            name + " rejected but not reported at INIT");
        continue;
      }
      for (int i = 0; i < 30; i++) {
        Pose pose =
            new Pose(
                10 + 60 * random.nextDouble(),
                10 + 120 * random.nextDouble(),
                Math.toRadians(360 * random.nextDouble()));
        Velocity v =
            new Velocity(-40 + 80 * random.nextDouble(), -40 + 80 * random.nextDouble(), 0);
        ShotSolution s =
            ShotSolver.solve(
                new ShotInputs(pose, v, field.target(HiveCell.A, 0), type), table, params, 0.4);
        if (s.isValid()) {
          assertTrue(
              Double.isFinite(s.targetRpm()) && s.targetRpm() > 0, "valid rpm " + s.targetRpm());
          assertTrue(
              s.targetHoodPosition() >= 0 && s.targetHoodPosition() <= 1,
              "valid hood " + s.targetHoodPosition());
          assertTrue(
              Double.isFinite(s.targetAzimuthRad()), "valid azimuth " + s.targetAzimuthRad());
          assertTrue(
              s.predictedFlightTimeSec() > 0 && Double.isFinite(s.predictedFlightTimeSec()),
              "valid flight time " + s.predictedFlightTimeSec());
        }
      }
    }
  }

  private static void hiveAndClockScenario(Random random) {
    long[] now = {10_000};
    HiveTracker hive = HiveTracker.fromConfig(() -> now[0]);
    for (int i = 0; i < 80; i++) {
      now[0] += random.nextInt(60);
      hive.observe(
          random.nextBoolean() ? HiveCell.A : HiveCell.B,
          random.nextBoolean(),
          now[0] - random.nextInt(50));
      assertEquals(hive.tips() % 2 == 0 ? HiveCell.A : HiveCell.B, hive.upCell());
    }
    MatchClock clock = MatchClock.fromConfig(MatchClock.Period.TELEOP, () -> now[0]);
    clock.start();
    for (int i = 0; i < 50; i++) {
      now[0] += 3_000;
      clock.pollCue();
      assertTrue(clock.remainingMs() >= 0, "negative time left");
    }
  }

  private static void sentinelScenario(Random random) {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Pose park = new Pose(16, 99, Math.toRadians(90));
    for (int i = 0; i < 5; i++) {
      Pose here = new Pose(15 + 50 * random.nextDouble(), 15 + 110 * random.nextDouble(), 0);
      sentinel.parkRoute(here, park);
      sentinel.isFullyOnAllianceSide(here);
      sentinel.earnsLeaveAndPark(here);
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Tuning generation

  @Provide
  Arbitrary<Map<String, Object>> tunings() {
    Arbitrary<Map.Entry<String, Object>> one =
        Arbitraries.of(KNOBS)
            .flatMap(
                k -> {
                  if (k.type() == boolean.class) {
                    return Arbitraries.of(true, false).map(v -> Map.entry(k.path(), (Object) v));
                  }
                  Arbitrary<Double> value =
                      Arbitraries.doubles().between(k.lo(), k.hi()).ofScale(4);
                  if (k.type() == int.class || k.type() == long.class) {
                    return value.map(v -> Map.entry(k.path(), (Object) (long) Math.round(v)));
                  }
                  return value.map(v -> Map.entry(k.path(), (Object) v));
                });
    return one.list()
        .ofMinSize(1)
        .ofMaxSize(10)
        .map(
            entries -> {
              Map<String, Object> m = new LinkedHashMap<>();
              entries.forEach(e -> m.put(e.getKey(), e.getValue()));
              return m;
            });
  }

  private static void apply(Map<String, Object> tuning) throws Exception {
    for (Map.Entry<String, Object> e : tuning.entrySet()) {
      String[] parts = e.getKey().split("\\.");
      Object holder = config.class.getField(parts[0]).get(null);
      for (int i = 1; i < parts.length - 1; i++) {
        holder = holder.getClass().getField(parts[i]).get(holder);
      }
      Field leaf = holder.getClass().getField(parts[parts.length - 1]);
      Object v = e.getValue();
      if (leaf.getType() == int.class) {
        leaf.setInt(holder, (int) (long) (Long) v);
      } else if (leaf.getType() == long.class) {
        leaf.setLong(holder, (Long) v);
      } else if (leaf.getType() == boolean.class) {
        leaf.setBoolean(holder, (Boolean) v);
      } else {
        leaf.setDouble(holder, ((Number) v).doubleValue());
      }
    }
  }

  @SuppressWarnings("unchecked")
  private static List<Knob> discoverKnobs() {
    Map<String, Object> docs;
    try (InputStream in =
        TuningFuzzProperties.class.getResourceAsStream(
            "/org/firstinspires/ftc/teamcode/robot/config/config-docs.yaml")) {
      docs = new Yaml().load(in);
    } catch (Exception e) {
      throw new IllegalStateException(e);
    }
    List<Knob> knobs = new ArrayList<>();
    try {
      for (String section : SECTIONS) {
        walk(config.class.getField(section).get(null), section, docs, knobs);
      }
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
    return knobs;
  }

  @SuppressWarnings("unchecked")
  private static void walk(Object node, String path, Map<String, Object> docs, List<Knob> knobs)
      throws IllegalAccessException {
    if (node == null || SKIP_PREFIXES.stream().anyMatch(path::startsWith)) {
      return;
    }
    for (Field f : node.getClass().getFields()) {
      if (java.lang.reflect.Modifier.isStatic(f.getModifiers())) {
        continue;
      }
      String child = path + "." + f.getName();
      if (SKIP_PREFIXES.stream().anyMatch(child::startsWith)) {
        continue;
      }
      Class<?> t = f.getType();
      if (t == boolean.class) {
        knobs.add(new Knob(child, t, 0, 1));
      } else if (t == double.class || t == int.class || t == long.class) {
        double current = ((Number) f.get(node)).doubleValue();
        Object doc = docs.get(f.getName());
        Double min = null;
        Double max = null;
        if (doc instanceof Map<?, ?> d) {
          min = d.get("min") instanceof Number n ? n.doubleValue() : null;
          max = d.get("max") instanceof Number n ? n.doubleValue() : null;
        }
        double span = Math.max(4 * Math.abs(current), 1.0);
        double lo = min != null ? min : (current >= 0 ? 0 : -span);
        double hi = max != null ? max : Math.max(lo + 1, span);
        // Rounded inward to the generator's 4 decimals, so the range stays inside the documented
        // one.
        knobs.add(new Knob(child, t, Math.ceil(lo * 1e4) / 1e4, Math.floor(hi * 1e4) / 1e4));
      } else if (!t.isPrimitive()
          && !t.isEnum()
          && t != String.class
          && !t.isArray()
          && !t.getName().startsWith("java.")
          && !t.getName().startsWith("com.pedropathing")) {
        walk(f.get(node), child, docs, knobs);
      }
    }
  }
}
