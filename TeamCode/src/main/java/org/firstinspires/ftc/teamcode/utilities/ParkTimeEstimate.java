package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.math.Pose;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * How long a park route takes to drive: a conservative estimate, not a prediction. Each leg is a
 * rest-to-rest trapezoidal profile at the slower (strafe) top speed, speeding up and slowing at the
 * measured natural deceleration, or the turn at {@code sentinel.park_turn_rate_dps} if that is
 * longer, plus {@code sentinel.park_settle_per_waypoint_sec}. Defense, a bumped path or a stall are
 * not modelled, which is why every user compares it with a margin.
 *
 * <p>Used by the TeleOp park alert and by the AUTO park-reserve tests, so both judge time the same
 * way.
 */
public final class ParkTimeEstimate {

  private ParkTimeEstimate() {}

  /**
   * Seconds to drive {@code route} from {@code here}. An empty route (no clear way to park) or a
   * non-finite pose is {@link Double#POSITIVE_INFINITY}: there is no time at which it is safe to
   * wait any longer.
   */
  public static double seconds(Pose here, List<Pose> route) {
    if (route.isEmpty() || !isFinite(here)) {
      return Double.POSITIVE_INFINITY;
    }
    var motion = config.pedro.motion;
    var sentinel = config.sentinel;
    double turnRate = Math.toRadians(sentinel.park_turn_rate_dps);
    double seconds = 0;
    Pose from = here;
    for (Pose to : route) {
      double turn = Math.abs(AngleUnit.normalizeRadians(to.heading() - from.heading()));
      double drive =
          driveSeconds(
              from.distance(to), motion.max_strafe_velocity, motion.natural_forward_deceleration);
      seconds += Math.max(drive, turn / turnRate) + sentinel.park_settle_per_waypoint_sec;
      from = to;
    }
    return seconds;
  }

  /**
   * Seconds to coast to a stop from {@code speedIps} (inches/second) at the slower measured natural
   * deceleration, before a park route that starts from rest can begin. Non-finite or negative speed
   * is infinite, so an unknown velocity never shortens the estimate.
   */
  public static double stoppingSeconds(double speedIps) {
    if (!(speedIps >= 0) || Double.isInfinite(speedIps)) {
      return Double.POSITIVE_INFINITY;
    }
    var motion = config.pedro.motion;
    double decel =
        Math.min(motion.natural_forward_deceleration, motion.natural_strafe_deceleration);
    return speedIps / decel;
  }

  /**
   * Rest-to-rest time over {@code distance} inches with top speed {@code v} and acceleration {@code
   * a}: a triangle profile when the distance is too short to reach {@code v} ({@code distance < v^2
   * / a}), otherwise a trapezoid.
   */
  static double driveSeconds(double distance, double v, double a) {
    if (distance <= 0) {
      return 0;
    }
    if (distance < v * v / a) {
      return 2 * Math.sqrt(distance / a);
    }
    return distance / v + v / a;
  }

  private static boolean isFinite(Pose p) {
    return p != null
        && Double.isFinite(p.x())
        && Double.isFinite(p.y())
        && Double.isFinite(p.heading());
  }
}
