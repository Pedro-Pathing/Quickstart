package org.firstinspires.ftc.teamcode.launch;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.helpers.Units;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.HiveCells;
import org.firstinspires.ftc.teamcode.vision.HiveTarget;

/**
 * Flywheel RPM against distance for one launcher, worked out with LaunchModel when it's created (at INIT),
 * so the OpMode loop only does a lookup. Distances are horizontal, from the launcher's axle to
 * LimelightCamera's target point.
 */
public class RpmTable {
    public static final double MIN_DISTANCE = 0.30;
    public static final double MAX_DISTANCE = 2.30;
    public static final double STEP = 0.05;
    /** Launch speed with neither the camera's target nor odometry to go on. */
    public static final double DEFAULT_LAUNCH_SPEED_MPS = 6.0;

    private final LaunchModel.Launcher launcher;
    private final double[] rpm;

    public RpmTable(LaunchModel.Launcher launcher) {
        this.launcher = launcher;
        int n = (int) Math.round((MAX_DISTANCE - MIN_DISTANCE) / STEP) + 1;
        rpm = new double[n];
        for (int i = 0; i < n; i++) {
            rpm[i] = LaunchModel.solveRPM(launcher, MIN_DISTANCE + i * STEP, LaunchModel.FACE_HEIGHT);
        }
        // Distances with no scoring RPM (e.g. too close) take the nearest distance the model solved
        double[] solved = rpm.clone();
        for (int i = 0; i < n; i++) {
            if (!Double.isNaN(solved[i])) continue;
            for (int d = 1; d < n; d++) {
                if (i - d >= 0 && !Double.isNaN(solved[i - d])) { rpm[i] = solved[i - d]; break; }
                if (i + d < n && !Double.isNaN(solved[i + d])) { rpm[i] = solved[i + d]; break; }
            }
        }
    }

    /**
     * RPM for our HIVE: for the camera's target if it has one, otherwise for odometry's distance from
     * this launcher's axle to our nearest CELL (HiveCells), as HiveAim falls back to odometry for the
     * heading. A fixed default speed instead, shots before the camera saw tags went about 0.5 m/s too
     * fast and long (the sim's shot log). With no pose either (no drive), the default speed.
     *
     * @param target the camera's target this loop, or null
     * @param pose   the robot's pose (Pedro field frame), or null
     */
    public double rpmFor(HiveTarget target, Alliance alliance, Pose pose) {
        if (target != null) return rpmFor(target);
        if (pose == null) return LaunchModel.rpmForSpeed(DEFAULT_LAUNCH_SPEED_MPS);
        double cos = Math.cos(pose.heading()), sin = Math.sin(pose.heading());
        double axleX = pose.x() + Units.metresToInches(launcher.axleForward * cos - launcher.axleLeft * sin);
        double axleY = pose.y() + Units.metresToInches(launcher.axleForward * sin + launcher.axleLeft * cos);
        double[] cell = HiveCells.nearestCell(alliance, axleX, axleY);
        return rpmAt(Units.inchesToMetres(Math.hypot(cell[0] - axleX, cell[1] - axleY)));
    }

    /** RPM to hit a target the camera sees, from this launcher's axle. */
    public double rpmFor(HiveTarget target) {
        return rpmAt(target.horizontalDistanceFrom(launcher.axleForward, launcher.axleLeft));
    }

    /** RPM for an axle-to-target distance, interpolated, held at the table's ends. NaN if nothing scores anywhere. */
    public double rpmAt(double distance) {
        double x = (distance - MIN_DISTANCE) / STEP;
        if (x <= 0) return rpm[0];
        if (x >= rpm.length - 1) return rpm[rpm.length - 1];
        int i = (int) Math.floor(x);
        double f = x - i;
        return rpm[i] * (1 - f) + rpm[i + 1] * f;
    }

    public double distanceAt(int i) {
        return MIN_DISTANCE + i * STEP;
    }

    public double rpmAtIndex(int i) {
        return rpm[i];
    }

    public int size() {
        return rpm.length;
    }
}
