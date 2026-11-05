package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.lifecycle.AfterProperty;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/**
 * Depth (X) and side (Y) protection must be mirror images of each other (code-quality rule 6). In a
 * world that is itself symmetric about the line y = x through the field centre (a square HIVE
 * frame, identical depth/side limits, identical forward/strafe braking), reflecting the whole
 * scenario across that line must reflect Casablanca's answer. Any wiring that treats X and Y
 * differently, such as a swapped bound or a velocity read from the wrong axis, breaks this for some
 * pose the generator finds.
 */
class CasablancaSymmetryProperties {
  private final CasablancaHarness harness = new CasablancaHarness();
  private final Casablanca casablanca;

  CasablancaSymmetryProperties() {
    config.field.hive.frame_depth_in = config.field.hive.frame_width_in;
    config.sentinel.protect_full_hive_footprint = true;
    config.pedro.braking.strafe.linear = config.pedro.braking.forward.linear;
    config.pedro.braking.strafe.quadratic = config.pedro.braking.forward.quadratic;

    casablanca = new Casablanca(new Sentinel(Alliance.RED));
    CasablancaHarness.isolateZoneProtection();
    Casablanca.sideSlowDown = Casablanca.depthSlowDown;
    Casablanca.sideHardStop = Casablanca.depthHardStop;
  }

  @AfterProperty
  void restore() {
    harness.restore();
  }

  /**
   * Reflection across y = x swaps field X and Y, maps heading h to pi/2 - h, reverses rotation (so
   * yaw rate and turn input change sign) and, in the robot frame, keeps forward but negates strafe.
   */
  @Property(tries = 3000)
  void reflectingTheScenarioReflectsTheCommand(
      @ForAll("nearHive") Pose pose,
      @ForAll @DoubleRange(min = -60, max = 60) double vx,
      @ForAll @DoubleRange(min = -60, max = 60) double vy,
      @ForAll @DoubleRange(min = -4, max = 4) double omega,
      @ForAll @DoubleRange(min = -1, max = 1) double strafe,
      @ForAll @DoubleRange(min = -1, max = 1) double forward,
      @ForAll @DoubleRange(min = -1, max = 1) double turn) {
    double[] out =
        casablanca.adjustDriveInput(pose, Vector2D.cartesian(vx, vy), omega, strafe, forward, turn);

    Pose reflected = new Pose(pose.y(), pose.x(), Math.PI / 2 - pose.heading());
    double[] reflectedOut =
        casablanca.adjustDriveInput(
            reflected, Vector2D.cartesian(vy, vx), -omega, -strafe, forward, -turn);

    // Output is {field Y, field X, turn}.
    assertEquals(out[1], reflectedOut[0], 1e-6, "field Y of the reflection is field X");
    assertEquals(out[0], reflectedOut[1], 1e-6, "field X of the reflection is field Y");
    assertEquals(-out[2], reflectedOut[2], 1e-9, "turn reverses");
  }

  @Provide
  Arbitrary<Pose> nearHive() {
    double c = config.field.size_in / 2.0;
    Arbitrary<Double> coord = Arbitraries.doubles().between(c - 45, c + 45).ofScale(6);
    Arbitrary<Double> heading =
        Arbitraries.doubles().between(-180, 180).ofScale(6).map(Math::toRadians);
    return Combinators.combine(coord, coord, heading).as(Pose::new);
  }
}
