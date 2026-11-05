package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * The ray calibration OpModes walk out along from a CELL opening, so every calibration shoots along
 * the same bearing and their results are comparable.
 *
 * <p>The ray leaves the opening on the side it faces, rotated {@code offAxisDeg} off its outward
 * direction. Straight out, the opening is only ~44 in from the audience wall; a 40 degree ray
 * reaches ~52 in while staying well inside the solver's off-axis gate. Distances are horizontal,
 * from the robot centre to the opening centre, exactly what {@code ShotSolver} looks up.
 */
public final class CalibrationRay {

  /** Gap (inches) kept between the robot footprint, at any heading, and the walls. */
  public static final double WALL_MARGIN_IN = 2.0;

  private final double targetX;
  private final double targetY;
  private final double bearingRad;
  private final double fieldSize;
  private final double wallClearance;

  public CalibrationRay(
      double targetX, double targetY, double bearingRad, double fieldSize, double wallClearance) {
    this.targetX = targetX;
    this.targetY = targetY;
    this.bearingRad = bearingRad;
    this.fieldSize = fieldSize;
    this.wallClearance = wallClearance;
  }

  /**
   * Robot centre to wall clearance: half the footprint's diagonal (a waypoint's heading points at
   * the CELL, so the robot is rarely square to a wall) plus {@link #WALL_MARGIN_IN}.
   */
  public static double wallClearanceFor(double robotWidthIn) {
    return robotWidthIn / Math.sqrt(2.0) + WALL_MARGIN_IN;
  }

  public double wallClearance() {
    return wallClearance;
  }

  /** Ray out from {@code cell}'s upward opening, {@code offAxisDeg} off its outward direction. */
  public static CalibrationRay forCell(FieldModel field, HiveCell cell, double offAxisDeg) {
    FieldModel.Opening opening = field.upOpening(cell);
    return new CalibrationRay(
        opening.x(),
        opening.y(),
        opening.facingRad() + Math.toRadians(offAxisDeg),
        field.size(),
        wallClearanceFor(config.sentinel.robot_width));
  }

  public double targetX() {
    return targetX;
  }

  public double targetY() {
    return targetY;
  }

  /** Unit vector pointing from the opening out along the ray, as {@code {x, y}}. */
  public double[] unit() {
    return new double[] {Math.cos(bearingRad), Math.sin(bearingRad)};
  }

  /**
   * Distance along the ray at which the robot centre first comes within {@link #wallClearance()} of
   * a wall. A requested distance past this is not calibratable along this bearing and must be
   * skipped, not clamped: clamping x and y independently slides the target off the ray and records
   * a shot at a distance it was never taken from.
   */
  public double maxDistance() {
    double[] u = unit();
    double limit = Double.MAX_VALUE;
    double low = wallClearance;
    double high = fieldSize - wallClearance;
    if (u[0] > 1e-6) {
      limit = Math.min(limit, (high - targetX) / u[0]);
    } else if (u[0] < -1e-6) {
      limit = Math.min(limit, (low - targetX) / u[0]);
    }
    if (u[1] > 1e-6) {
      limit = Math.min(limit, (high - targetY) / u[1]);
    } else if (u[1] < -1e-6) {
      limit = Math.min(limit, (low - targetY) / u[1]);
    }
    return Math.max(0.0, limit);
  }

  /** Pose {@code distanceInches} out along the ray, headed back at the opening. */
  public Pose waypoint(double distanceInches) {
    double[] u = unit();
    double x = targetX + distanceInches * u[0];
    double y = targetY + distanceInches * u[1];
    return new Pose(x, y, Math.atan2(targetY - y, targetX - x));
  }
}
