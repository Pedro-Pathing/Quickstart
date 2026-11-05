package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Test;

/**
 * The Casablanca autotuners reset odometry to their configured calibration_pose and abort a trial
 * the moment HIVE protection intervenes. From that pose, every motion a trial makes must therefore
 * leave protection completely inactive, or no trial can ever finish ("No passing candidate"
 * forever, however the robot is placed). The shipped default once sat inside the HIVE frame.
 */
public class TunerPoseSafetyTest {

  /** Drives the pose along the robot's forward or strafe axis and checks protection never acts. */
  private static void assertDrivesFreely(Pose start, double travel, boolean strafe, String what) {
    Casablanca casablanca = new Casablanca(new Sentinel(Alliance.RED));
    Casablanca.enableInputSmoothing = false;
    for (double sign : new double[] {1, -1}) {
      Vector2D axis = (strafe ? Vector2D.cartesian(0, sign) : Vector2D.cartesian(sign, 0));
      Vector2D fieldStep = axis.rotate(start.heading());
      for (double d = 0; d <= travel; d += 1.0) {
        Pose pose =
            new Pose(start.x() + fieldStep.x() * d, start.y() + fieldStep.y() * d, start.heading());
        Vector2D velocity = Vector2D.cartesian(fieldStep.x() * 40, fieldStep.y() * 40);
        casablanca.adjustDriveInput(pose, velocity, 0, axis.y(), axis.x(), 0);
        String where = what + (strafe ? " strafe " : " forward ") + sign * d + " in";
        assertEquals(1.0, casablanca.getLastDepthScale(), 0.0, where + ": depth protection acted");
        assertEquals(1.0, casablanca.getLastSideScale(), 0.0, where + ": side protection acted");
        assertFalse(casablanca.getLastDepthRepulsion() || casablanca.getLastSideRepulsion(), where);
      }
    }
  }

  @Test
  public void thePodContactTunerCanDriveItsFullTravelFromItsPose() {
    var t = config.casablanca.jump_tuner;
    Pose p = t.calibration_pose;
    assertDrivesFreely(p, t.max_travel_inches, false, "jump tuner");
    // STRAFE trials start at the same spot turned 90 degrees, so they strafe along the same lane.
    Pose strafeStart = new Pose(p.x(), p.y(), p.heading() - Math.PI / 2);
    assertDrivesFreely(strafeStart, t.max_travel_inches, true, "jump tuner");
  }

  @Test
  public void theHeadingTunerCanSweepAndDriftFromItsPose() {
    var t = config.casablanca.heading_tuner;
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Pose start = t.calibration_pose;
    double sweep = Math.toRadians(t.maximum_sweep_degrees);
    for (double dx : new double[] {-t.position_drift_inches, 0, t.position_drift_inches}) {
      for (double dy : new double[] {-t.position_drift_inches, 0, t.position_drift_inches}) {
        Pose p = new Pose(start.x() + dx, start.y() + dy, start.heading());
        assertTrue(sentinel.isClearOfHive(p), "heading tuner pose " + p + " touches the HIVE");
        assertTrue(sentinel.isRotationClear(p, start.heading() + sweep), "sweep left from " + p);
        assertTrue(sentinel.isRotationClear(p, start.heading() - sweep), "sweep right from " + p);
      }
    }
    assertDrivesFreely(start, t.position_drift_inches, false, "heading tuner");
    assertDrivesFreely(start, t.position_drift_inches, true, "heading tuner");
  }
}
