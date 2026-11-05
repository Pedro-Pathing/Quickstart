package org.firstinspires.ftc.teamcode.records;

import androidx.annotation.Nullable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/**
 * Immutable snapshot fed to the shot solver. Pose values are copied into new {@link Pose} objects
 * by the publisher, so the solver thread never reads a pose the control loop is mutating.
 */
public record ShotInputs(
    Pose robotPose, // Localizer pose (x, y in inches; heading in radians)
    Velocity robotVelocity, // Field-frame velocity (vx, vy in in/s; omega in rad/s, CCW positive)
    CellTarget target, // Upward-facing CELL opening to aim at
    BallType ballType // Which shot table to use
    ) {
  public static boolean isFinitePose(@Nullable Pose pose) {
    return pose != null
        && Double.isFinite(pose.x())
        && Double.isFinite(pose.y())
        && Double.isFinite(pose.heading());
  }

  public static boolean isFiniteVelocity(@Nullable Velocity velocity) {
    return velocity != null
        && Double.isFinite(velocity.vx)
        && Double.isFinite(velocity.vy)
        && Double.isFinite(velocity.omega);
  }
}
