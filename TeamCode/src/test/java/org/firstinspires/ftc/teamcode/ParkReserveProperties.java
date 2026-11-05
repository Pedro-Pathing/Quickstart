package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertFalse;

import com.pedropathing.math.Pose;
import java.util.List;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/**
 * AUTO's park guard cancels whatever the routine is doing at its park deadline (the estimated
 * drive, {@link ParkTimeEstimate}, plus {@code auto.park_margin_ms}) and drives {@link
 * Sentinel#parkRoute} to the park pose for LEAVE + PARK. From anywhere the robot can legally be,
 * even pushed flush into a wall, that route must exist, or the deadline has nothing to drive.
 */
class ParkReserveProperties {

  @Property(tries = 1000)
  void fromAnyLegalPoseAParkRouteExists(
      @ForAll Alliance alliance, @ForAll("redSidePoses") Pose redFramePose) {
    FieldModel field = new FieldModel(alliance);
    Sentinel sentinel = new Sentinel(field);
    Pose here = field.fromRedFrame(redFramePose);
    if (!sentinel.routeProblem(List.of(here), true).isEmpty()) {
      return; // Not somewhere a legal routine can be.
    }
    Pose park = MatchProfile.loadMatchProfile(alliance).parkPose();

    List<Pose> route = sentinel.parkRoute(here, park);
    assertFalse(route.isEmpty(), () -> alliance + " has no park route from " + here);
  }

  @Provide
  Arbitrary<Pose> redSidePoses() {
    double size = config.field.size_in;
    Arbitrary<Double> x = Arbitraries.doubles().between(9, size / 2).ofScale(2);
    Arbitrary<Double> y = Arbitraries.doubles().between(9, size - 9).ofScale(2);
    Arbitrary<Double> h = Arbitraries.doubles().between(-180, 180).ofScale(1);
    return Combinators.combine(x, y, h).as((px, py, ph) -> new Pose(px, py, Math.toRadians(ph)));
  }
}
