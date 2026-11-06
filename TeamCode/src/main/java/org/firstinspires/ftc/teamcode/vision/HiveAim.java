package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Chooses the field heading that points the robot at our HIVE, for auto-aim in TeleOp and Autonomous.
 *
 * With tags in view it points straight at the target CELL's open face (the camera's estimate).
 * Before the camera has seen tags it points roughly at our nearest CELL using odometry. Once the
 * camera has seen the target, the aim remembers where that target is on the field, and keeps
 * pointing at that spot even after the tags drop out. Call reset() when a new aim starts.
 *
 * Each camera image gives the target's position relative to where the robot was when the image was
 * taken. That is turned into a fixed point on the field once, using the robot's full pose (x, y and
 * heading) at capture time from the pose history, because the CELL doesn't move when the robot
 * does. Every loop the aim heading is then the direction from the robot's current position to that
 * point. Remembering a heading instead would go stale as soon as the robot drove somewhere else
 * with the tags out of view, and adding the camera's bearing to the heading now on every loop would
 * run ahead of the robot as it turns and overshoot. Call update() every loop, aiming or not, to keep the history.
 *
 * The aim line goes through the robot's centre, with no allowance for where the launchers sit.
 */
@Configurable
public class HiveAim {
    /** The aim is locked (ready to shoot) within this of the aim heading... */
    public static double lockToleranceDegrees = 2.0;
    /** ...while turning slower than this (rad/s). Shared by TeleOp and the autos. */
    public static double lockSettleRate = 0.3;

    private static final double METRES_TO_INCHES = 1.0 / 0.0254;
    /**
     * Closer than this (inches) to the remembered point the direction to it swings wildly for a small
     * move, so the last heading is held instead. The point is kept, so the aim recovers on leaving.
     */
    static final double MIN_RANGE_INCHES = 4.0;

    private final PoseHistory history = new PoseHistory();
    private boolean fromVision = false;
    private long frame = -1;
    /** The remembered target, in Pedro field inches. Only meaningful while fromVision is true. */
    private double targetX, targetY;
    private double heading = 0.0;

    /** Start a new aim: odometry until the camera sees tags. */
    public void reset() {
        fromVision = false;
        frame = -1;
    }

    /**
     * The heading to aim at this loop.
     *
     * @param target the camera's target this loop, or null if no tags are in view
     * @param pose   the robot's pose this loop (Pedro field frame)
     */
    public double update(Alliance alliance, HiveTarget target, Pose pose) {
        return update(alliance, target, pose, System.nanoTime());
    }

    /** As update(), with the time the pose is recorded at given, so tests can control it. */
    double update(Alliance alliance, HiveTarget target, Pose pose, long nowNanos) {
        history.record(nowNanos, pose);
        if (target != null && target.frame != frame) {
            Pose then = history.at(target.captureNanos);
            double cos = Math.cos(then.heading()), sin = Math.sin(then.heading());
            targetX = then.x() + (target.x * cos - target.y * sin) * METRES_TO_INCHES;
            targetY = then.y() + (target.x * sin + target.y * cos) * METRES_TO_INCHES;
            frame = target.frame;
            fromVision = true;
        }
        if (fromVision) {
            double dx = targetX - pose.x(), dy = targetY - pose.y();
            if (Math.hypot(dx, dy) >= MIN_RANGE_INCHES) heading = Math.atan2(dy, dx);
        } else {
            heading = HiveCells.headingToNearestCell(alliance, pose.x(), pose.y());
        }
        return heading;
    }

    /** Whether a heading error (radians) and turn rate (rad/s) count as locked on. */
    public static boolean isLocked(double headingError, double turnRate) {
        return Math.abs(headingError) < Math.toRadians(lockToleranceDegrees) && Math.abs(turnRate) < lockSettleRate;
    }

    /** Whether this aim has had a heading from the camera yet. */
    public boolean isFromVision() {
        return fromVision;
    }
}
