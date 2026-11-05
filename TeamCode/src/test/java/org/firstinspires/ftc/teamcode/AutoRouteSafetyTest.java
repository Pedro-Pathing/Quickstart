package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Checks every configured routine and emergency parking from positions along each route.
 *
 * <p>This is the only route check: the robot no longer re-checks routes at INIT. It runs with every
 * APK build and, tagged {@code push-config}, in {@code checkPushConfig} before {@code pushConfig}
 * sends a config to the robot.
 */
@Tag("push-config")
public class AutoRouteSafetyTest {
  @Test
  public void startsAndParksMeetTheFieldConstraints() {
    for (Alliance alliance : Alliance.values()) {
      Sentinel sentinel = new Sentinel(alliance);
      for (var routine : AutoFixtures.routines(alliance)) {
        String name = routine.toString();
        assertTrue(
            sentinel.isFullyOnAllianceSide(routine.start()), name + " start on alliance side");
        assertTrue(sentinel.isTouchingWall(routine.start()), name + " start touching wall");
        assertFalse(sentinel.isParked(routine.start()), name + " start outside loading zone");
        assertTrue(sentinel.isClearOfHive(routine.start()), name + " start clear of HIVE");
        assertTrue(sentinel.earnsLeaveAndPark(routine.park()), name + " park earns LEAVE and PARK");
      }
    }
  }

  @Test
  public void everyConfiguredPathStaysClearAndOnItsAllianceSide() {
    for (Alliance alliance : Alliance.values()) {
      Sentinel sentinel = new Sentinel(alliance);
      for (var routine : AutoFixtures.routines(alliance)) {
        routine.paths().forEach((name, path) -> assertSafe(sentinel, routine + " " + name, path));
      }
    }
  }

  @Test
  public void parkGuardCanSafelyExitEveryConfiguredRoute() {
    for (Alliance alliance : Alliance.values()) {
      Sentinel sentinel = new Sentinel(alliance);
      for (var routine : AutoFixtures.routines(alliance)) {
        routine
            .paths()
            .forEach(
                (name, path) -> {
                  for (int i = 0; i <= 20; i++) {
                    Pose from = path.get(i / 20.0);
                    var route = sentinel.parkRoute(from, routine.park());
                    assertFalse(
                        route.isEmpty(), routine + " " + name + " has a park route at " + i);
                    for (Pose to : route) {
                      if (from.distance(to) > 0.0) {
                        assertSafe(
                            sentinel, routine + " " + name + " park at " + i, pline(from, to));
                      }
                      from = to;
                    }
                  }
                });
      }
    }
  }

  /**
   * Every volley pose must point the robot close enough at its CELL that the turret can cover the
   * rest (and a turret-less robot is already facing it): a pose that looks fine on the field map
   * can still leave the target outside the turret's travel.
   */
  @Test
  public void everyVolleyPoseFacesItsCellWithinTurretReach() {
    double reachDeg = Math.min(-config.turret.travel.min_angle, config.turret.travel.max_angle);
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      for (AutoFixtures.Volley v : AutoFixtures.allVolleys(alliance)) {
        FieldModel.Opening o = field.upOpening(v.cell());
        double bearing = Math.atan2(o.y() - v.pose().y(), o.x() - v.pose().x());
        double errorDeg = Math.toDegrees(AngleUnit.normalizeRadians(bearing - v.pose().heading()));
        assertTrue(
            Math.abs(errorDeg) <= reachDeg,
            alliance + " " + v.name() + " is " + errorDeg + " deg off its CELL");
      }
    }
  }

  private static void assertSafe(Sentinel sentinel, String name, Path path) {
    var problem = sentinel.routeProblem(path, 0.005, true);
    assertTrue(problem.isEmpty(), name + ": " + problem.orElse("clear"));
  }
}
