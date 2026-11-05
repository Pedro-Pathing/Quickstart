package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Test;

public class RotationSweepSafetyTest {
  @Test
  public void lookaheadRejectsAnIntermediateCollisionEvenWhenBothEndsAreClear() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      Sentinel sentinel = new Sentinel(field);
      var zone = new FieldModel(Alliance.RED).hiveFrame();
      // Endpoints at 35/55 degrees have narrower X extent than the corner at 45 degrees.
      double radius = sentinel.robotWidth() / Math.sqrt(2);
      double endpointExtent =
          sentinel.robotWidth() / 2 * (Math.cos(Math.toRadians(35)) + Math.sin(Math.toRadians(35)));
      double centreX = zone.getMinX() - (radius + endpointExtent) / 2;
      for (double sign : new double[] {-1, 1}) {
        Pose from =
            field.fromRedFrame(new Pose(centreX, zone.centre().y, sign * Math.toRadians(35)));
        Pose to = field.fromRedFrame(new Pose(centreX, zone.centre().y, sign * Math.toRadians(55)));
        assertTrue(sentinel.isClearOfHive(from));
        assertTrue(sentinel.isClearOfHive(to));
        assertFalse(sentinel.isRotationClear(from, to.heading()));
        assertFalse(
            sentinel.isRotationSafe(from, sign, Math.toRadians(20)),
            "manual drive must check the swept corner too");
        assertTrue(
            sentinel.isRotationSafe(from, -sign, Math.toRadians(20)),
            "turning away remains possible");
      }
    }
  }
}
