package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class CasablancaBrakingFrameTest {
  private boolean friction;
  private boolean smoothing;
  private boolean headingLock;
  private boolean fieldCentric;
  private double forwardLinear;
  private double forwardQuadratic;
  private double strafeLinear;
  private double strafeQuadratic;
  private Sentinel sentinel;
  private Casablanca casablanca;

  @BeforeEach
  public void setUp() {
    friction = Casablanca.enableFrictionComp;
    smoothing = Casablanca.enableInputSmoothing;
    headingLock = Casablanca.enableHeadingLock;
    fieldCentric = Casablanca.fieldCentric;
    forwardLinear = config.pedro.braking.forward.linear;
    forwardQuadratic = config.pedro.braking.forward.quadratic;
    strafeLinear = config.pedro.braking.strafe.linear;
    strafeQuadratic = config.pedro.braking.strafe.quadratic;
    sentinel = new Sentinel(Alliance.RED);
    casablanca = new Casablanca(sentinel);
    Casablanca.enableFrictionComp = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableHeadingLock = false;
    Casablanca.fieldCentric = false;
  }

  @AfterEach
  public void restore() {
    config.pedro.braking.forward.linear = forwardLinear;
    config.pedro.braking.forward.quadratic = forwardQuadratic;
    config.pedro.braking.strafe.linear = strafeLinear;
    config.pedro.braking.strafe.quadratic = strafeQuadratic;
    Casablanca.enableFrictionComp = friction;
    Casablanca.enableInputSmoothing = smoothing;
    Casablanca.enableHeadingLock = headingLock;
    Casablanca.fieldCentric = fieldCentric;
  }

  @Test
  public void headingZeroPreservesTheOriginalStoppingDistancesExactly() {
    for (double sign : new double[] {-1, 0, 1}) {
      Vector2D stop = Casablanca.fieldStoppingDistance(Vector2D.cartesian(sign * 30, sign * 20), 0);
      assertEquals(
          Casablanca.brakingDistance(sign * 30, forwardLinear, forwardQuadratic), stop.x(), 0.0);
      assertEquals(
          Casablanca.brakingDistance(sign * 20, strafeLinear, strafeQuadratic), stop.y(), 0.0);
    }
  }

  @Test
  public void quarterTurnUsesForwardBrakingOnFieldYAndStrafeOnFieldX() {
    for (double heading : new double[] {Math.PI / 2, -Math.PI / 2}) {
      for (double sign : new double[] {-1, 1}) {
        Vector2D stop =
            Casablanca.fieldStoppingDistance(Vector2D.cartesian(sign * 30, sign * 20), heading);
        assertEquals(
            Casablanca.brakingDistance(sign * 30, strafeLinear, strafeQuadratic), stop.x(), 1e-12);
        assertEquals(
            Casablanca.brakingDistance(sign * 20, forwardLinear, forwardQuadratic),
            stop.y(),
            1e-12);
      }
    }
  }

  @Test
  public void diagonalStopVectorRotatesBackToTheField() {
    Vector2D robotVelocity = Vector2D.cartesian(-25, 40);
    double heading = Math.PI / 4;
    Vector2D expected =
        Vector2D.cartesian(
                Casablanca.brakingDistance(-25, forwardLinear, forwardQuadratic),
                Casablanca.brakingDistance(40, strafeLinear, strafeQuadratic))
            .rotate(heading);
    Vector2D actual = Casablanca.fieldStoppingDistance(robotVelocity.rotate(heading), heading);
    assertEquals(expected.x(), actual.x(), 1e-12);
    assertEquals(expected.y(), actual.y(), 1e-12);
  }

  @Test
  public void driveProtectionUsesProjectedStoppingDistanceOnBothAxes() {
    config.pedro.braking.forward.linear = 1.0;
    config.pedro.braking.forward.quadratic = 0.0;
    config.pedro.braking.strafe.linear = 0.25;
    config.pedro.braking.strafe.quadratic = 0.0;
    var zone = sentinel.getProtectedZones().get(0);
    double speed = 60;
    for (double heading : new double[] {0, Math.PI / 2}) {
      for (boolean xAxis : new boolean[] {true, false}) {
        double hard = xAxis ? Casablanca.depthHardStop : Casablanca.sideHardStop;
        double slow = xAxis ? Casablanca.depthSlowDown : Casablanca.sideSlowDown;
        double gap = hard + (slow - hard) / 2;
        Pose origin = new Pose(0, 0, heading);
        var bounds = sentinel.getRobotBounds(origin);
        Pose pose =
            xAxis
                ? new Pose(zone.getMinX() - bounds.getMaxX() - gap, zone.centre().y, heading)
                : new Pose(zone.centre().x, zone.getMinY() - bounds.getMaxY() - gap, heading);
        Vector2D velocity = xAxis ? Vector2D.cartesian(speed, 0) : Vector2D.cartesian(0, speed);
        Vector2D input = velocity.normalized().rotate(-heading);
        double[] output = casablanca.adjustDriveInput(pose, velocity, 0, input.y(), input.x(), 0);
        boolean forwardAxis = xAxis == (heading == 0);
        double stop = speed * (forwardAxis ? 1.0 : 0.25);
        double expected =
            Math.min(0.5, Math.sqrt((gap - hard) * Casablanca.decelSafetyFactor / stop));
        assertEquals(
            expected, output[xAxis ? 1 : 0], 1e-12, "axis=" + xAxis + ", heading=" + heading);
      }
    }
  }
}
