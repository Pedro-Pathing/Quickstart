package org.firstinspires.ftc.teamcode.ballistics;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;

/**
 * Stateless shot solve against an upward-facing HIVE CELL opening.
 *
 * <p>Hood and flywheel come from the measured {@link ShotTable} for the SCORING ELEMENT being shot,
 * looked up by horizontal distance to the opening centre (the target height is fixed for every
 * upward CELL, so distance is the only free variable). On top of the table lookup the solver
 * enforces what is specific to BIOBUZZ:
 *
 * <ul>
 *   <li>Approach side. A CELL opening faces outward and 30 degrees up, so a ball can only enter
 *       from roughly in front of it. A robot behind or far beside the opening is refused rather
 *       than lobbing into the outside of the CELL (a G417 risk).
 *   <li>Calibration. Placeholder tables ({@code shooter.shot_table.calibrated: false}) never yield
 *       a valid solution.
 *   <li>Identity. Each solution carries the HIVE epoch of its target, so the controller can refuse
 *       to fire a solution computed before a TIP.
 * </ul>
 *
 * <p>Moving shots lead the target by the robot's velocity times the estimated time of flight.
 */
public final class ShotSolver {

  private ShotSolver() {}

  public static ShotSolution solve(
      ShotInputs inputs, ShotTable table, BallisticsParameters params, double lastFlightTime) {
    if (inputs == null || inputs.robotPose() == null || table == null || params == null) {
      return ShotSolution.invalid("Null inputs, table, or parameters");
    }
    CellTarget target = inputs.target();
    if (target == null) {
      return ShotSolution.invalid("No upward CELL to aim at");
    }

    Pose pose = inputs.robotPose();
    Velocity vel = inputs.robotVelocity() != null ? inputs.robotVelocity() : Velocity.zero();
    if (!ShotInputs.isFinitePose(pose) || !ShotInputs.isFiniteVelocity(vel)) {
      return ShotSolution.invalid("Non-finite robot pose or velocity");
    }
    if (!Double.isFinite(target.x())
        || !Double.isFinite(target.y())
        || !Double.isFinite(target.heightInches())
        || !Double.isFinite(target.facingRad())) {
      return ShotSolution.invalid("Non-finite CELL target");
    }

    double rx = pose.x();
    double ry = pose.y();
    double gx = target.x();
    double gy = target.y();
    int epoch = target.epoch();

    double vx = vel.vx;
    double vy = vel.vy;
    double robotSpeed = Math.hypot(vx, vy);
    double directDistance = Math.hypot(gx - rx, gy - ry);

    double flightTime =
        (lastFlightTime > 0.05 && lastFlightTime < 3.0)
            ? lastFlightTime
            : params.flightTimeAt(directDistance);
    double leadWeight = leadWeight(robotSpeed, params.leadMinSpeedIps());
    double virtX = gx;
    double virtY = gy;
    double virtDist = directDistance;

    for (int iter = 0; iter < 3; iter++) {
      double lead = leadWeight * flightTime * params.leadBiasGain();
      virtX = gx - vx * lead;
      virtY = gy - vy * lead;
      virtDist = Math.hypot(virtY - ry, virtX - rx);

      double next = params.flightTimeAt(virtDist);
      boolean converged = Math.abs(next - flightTime) < 1e-3;
      flightTime = next;
      if (converged || leadWeight == 0.0) {
        break;
      }
    }

    ShotTable.Shot shot = table.lookup(virtDist);
    double offAxisDeg = Math.toDegrees(offAxisRad(rx, ry, target));

    String reason = "Valid";
    boolean valid = true;
    if (robotSpeed > params.maxMovingSpeedIps()) {
      // Still aimed with lead above, so the turret is already right when the robot slows down.
      valid = false;
      reason =
          String.format(
              Locale.ROOT,
              "Robot speed %.1f in/s exceeds max speed gate %.1f in/s",
              robotSpeed,
              params.maxMovingSpeedIps());
    } else if (offAxisDeg > params.maxOffAxisDeg()) {
      valid = false;
      reason =
          String.format(
              Locale.ROOT,
              "%.0f deg off CELL %s's opening axis (max %.0f): move in front of the opening",
              offAxisDeg,
              target.cell(),
              params.maxOffAxisDeg());
    } else if (virtDist < params.minValidDistanceInches()
        || virtDist > params.maxValidDistanceInches()) {
      valid = false;
      reason =
          String.format(
              Locale.ROOT,
              "Distance %.1f in outside calibrated range [%.1f, %.1f]",
              virtDist,
              params.minValidDistanceInches(),
              params.maxValidDistanceInches());
    } else if (!shot.withinCalibratedRange()) {
      valid = false;
      reason = shot.reason();
    } else if (!table.isCalibrated()) {
      valid = false;
      reason = "UNCALIBRATED: shooter.shot_table.calibrated is false (placeholder rows)";
    }

    double azimuthRad = Math.atan2(virtY - ry, virtX - rx);

    return new ShotSolution(
        shot.hoodServoPosition(),
        shot.rpm(),
        params.rpmToV0(shot.rpm()),
        azimuthRad,
        flightTime,
        0.0,
        virtDist,
        valid,
        reason,
        epoch,
        inputs.ballType());
  }

  /**
   * How much of the moving-shot lead to apply: none below {@code leadMinSpeedIps} (estimator noise
   * on a robot that is holding still), rising linearly to all of it at twice that speed. A hard
   * on/off switch made the aim jump by the whole lead angle as the speed crossed the threshold;
   * with ~0.7 s lob flights that was over 10 degrees at 10 in/s.
   */
  public static double leadWeight(double speedIps, double leadMinSpeedIps) {
    if (leadMinSpeedIps <= 0.0) {
      return 1.0;
    }
    return Math.clamp((speedIps - leadMinSpeedIps) / leadMinSpeedIps, 0.0, 1.0);
  }

  /**
   * Angle between the CELL opening's outward direction and the line from the opening to the robot:
   * 0 straight in front, pi directly behind. The opening faces outward and up, so only small angles
   * can put a ball through it.
   */
  public static double offAxisRad(double robotX, double robotY, CellTarget target) {
    double dx = robotX - target.x();
    double dy = robotY - target.y();
    if (Math.hypot(dx, dy) < 1e-6) {
      return Math.PI;
    }
    return Math.abs(AngleUnit.normalizeRadians(Math.atan2(dy, dx) - target.facingRad()));
  }
}
