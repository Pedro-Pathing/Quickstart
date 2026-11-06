package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.vision.HiveTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

/**
 * Whether the flywheels have a distance worth shooting on: the camera is up and settled, and has
 * seen tags since it settled (so the RPM tables have the camera's distance), or it has been up
 * for tagWaitSeconds without seeing any (then the odometry RPM is used). Call update() every loop.
 */
@Configurable
public class ShotReadiness {
    /** With the camera up but no tags in view for this long, shoot on the odometry RPM. */
    public static double tagWaitSeconds = 0.75;

    private long settledSinceNanos = -1;
    private boolean tagsSeen = false;

    public void update(LimelightCamera camera, HiveTarget target) {
        if (!camera.isAvailable()) {
            // No camera: odometry is all there is
            if (settledSinceNanos < 0) settledSinceNanos = System.nanoTime();
            return;
        }
        if (camera.getMode() != LimelightCamera.Mode.TAGS || !camera.isSettled()) {
            settledSinceNanos = -1;
            tagsSeen = false;
            return;
        }
        if (settledSinceNanos < 0) {
            settledSinceNanos = System.nanoTime();
            tagsSeen = false;
        }
        if (target != null && target.captureNanos > settledSinceNanos) tagsSeen = true;
    }

    public boolean isReady() {
        if (settledSinceNanos < 0) return false;
        return tagsSeen || System.nanoTime() - settledSinceNanos > (long) (tagWaitSeconds * 1e9);
    }

    /** Whether the distance is the camera's (tags seen since it settled) rather than odometry's. */
    public boolean fromTags() {
        return tagsSeen;
    }
}
