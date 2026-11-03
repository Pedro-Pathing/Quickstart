package org.firstinspires.ftc.teamcode.vision;

/**
 * Where the centre of the target CELL's open face is, relative to the robot, at the moment the
 * Limelight captured the image.
 *
 * Robot frame: +X forward, +Y left, +Z up, origin at the robot's centre of rotation. Metres and radians.
 */
public class HiveTarget {
    public enum Cell { SCORING, AUDIENCE }

    /** Which CELL of our HIVE the target is. */
    public final Cell cell;
    /** Target position in the robot frame. */
    public final double x, y, z;
    /** How many of the CELL's tags were averaged. */
    public final int tagCount;
    /** The largest tag in view, which chose the CELL. */
    public final int largestTagId;
    /** Age of the camera result in milliseconds. */
    public final long stalenessMs;
    /** Which camera frame this came from: the same number means the same image. */
    public final long frame;
    /** When the image was captured (System.nanoTime()), for pairing it with the robot's pose then. */
    public final long captureNanos;

    public HiveTarget(Cell cell, double x, double y, double z, int tagCount, int largestTagId, long stalenessMs,
                      long frame, long captureNanos) {
        this.cell = cell;
        this.x = x;
        this.y = y;
        this.z = z;
        this.tagCount = tagCount;
        this.largestTagId = largestTagId;
        this.stalenessMs = stalenessMs;
        this.frame = frame;
        this.captureNanos = captureNanos;
    }

    /** Angle from the robot's forward direction to the target, positive to the left (CCW). */
    public double bearing() {
        return Math.atan2(y, x);
    }

    /** Horizontal distance from the robot's centre to the target. */
    public double horizontalDistance() {
        return Math.hypot(x, y);
    }

    /** Horizontal distance from a point on the robot (e.g. a launcher axle) to the target. */
    public double horizontalDistanceFrom(double forward, double left) {
        return Math.hypot(x - forward, y - left);
    }

    /** Field heading that points the robot's centre line straight at the target. */
    public double targetHeading(double currentHeading) {
        return targetHeading(currentHeading, 0.0);
    }

    /**
     * Field heading that makes a launcher, mounted parallel to the robot's centre line and
     * launcherLeft to the left of it, fire straight through the target.
     *
     * @param currentHeading the robot's field heading when the image was captured (radians, CCW positive)
     */
    public double targetHeading(double currentHeading, double launcherLeft) {
        double range = horizontalDistance();
        if (range < 1e-6) return currentHeading;
        // Turning by theta puts the target at a sideways offset of range * sin(bearing - theta),
        // which must equal launcherLeft for the launcher's line to pass through it
        double ratio = Math.max(-1.0, Math.min(1.0, launcherLeft / range));
        double turn = bearing() - Math.asin(ratio);
        return normalize(currentHeading + turn);
    }

    private static double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
