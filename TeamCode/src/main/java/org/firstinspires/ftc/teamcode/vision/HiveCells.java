package org.firstinspires.ftc.teamcode.vision;

/**
 * Rough field positions of the HIVE CELLs, for aiming with odometry before the camera sees tags.
 *
 * Pedro field frame, inches: (72, 72) is the field centre, +X towards the blue alliance, +Y towards
 * the scoring side (away from the audience), heading counter-clockwise from +X. This is the frame the
 * BIOBUZZ sim feeds TeamCode, so the robot's start pose must be set in the same frame.
 *
 * From the field CAD: the HIVEs are 12.74 in either side of the centre line. A raised CELL's face is
 * 15.6 in from the centre and a lowered one 19.9 in, and tipping swaps them, so each CELL is placed
 * midway at 17.8 in. That's at most about 2 in out, which the camera corrects.
 */
public class HiveCells {
    public static final double RED_HIVE_X = 72 - 12.74;
    public static final double BLUE_HIVE_X = 72 + 12.74;
    public static final double AUDIENCE_CELL_Y = 72 - 17.8;
    public static final double SCORING_CELL_Y = 72 + 17.8;

    /** {x, y} of our alliance's CELL nearest to the given field position. */
    public static double[] nearestCell(Alliance alliance, double x, double y) {
        double hiveX = alliance == Alliance.RED ? RED_HIVE_X : BLUE_HIVE_X;
        double audience = Math.hypot(hiveX - x, AUDIENCE_CELL_Y - y);
        double scoring = Math.hypot(hiveX - x, SCORING_CELL_Y - y);
        return new double[] {hiveX, audience <= scoring ? AUDIENCE_CELL_Y : SCORING_CELL_Y};
    }

    /** Field heading (radians, CCW from +X) from the given position to our alliance's nearest CELL. */
    public static double headingToNearestCell(Alliance alliance, double x, double y) {
        double[] cell = nearestCell(alliance, x, y);
        return Math.atan2(cell[1] - y, cell[0] - x);
    }
}
