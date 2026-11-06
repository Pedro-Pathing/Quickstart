package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;

/**
 * The robot's recent poses, so a camera result can be paired with where the robot was when the
 * image was captured rather than where it is now. Record the pose every loop.
 */
public class PoseHistory {
    /** Long enough to cover the camera's latency and the staleness limit. */
    private static final int SIZE = 64;
    private static final long KEEP_NANOS = 500_000_000L;

    private final long[] times = new long[SIZE];
    private final Pose[] poses = new Pose[SIZE];
    private int newest = -1;
    private int count = 0;

    public void record(Pose pose) {
        record(System.nanoTime(), pose);
    }

    public void record(long timeNanos, Pose pose) {
        newest = (newest + 1) % SIZE;
        times[newest] = timeNanos;
        poses[newest] = pose;
        count = Math.min(count + 1, SIZE);
    }

    /**
     * The pose at that time, interpolated between the recorded poses either side. Before the oldest
     * kept pose it's the oldest; after the newest it's the newest. Null if nothing is recorded.
     */
    public Pose at(long timeNanos) {
        if (count == 0) return null;
        Pose later = poses[newest];
        long laterTime = times[newest];
        if (timeNanos >= laterTime) return later;
        for (int i = 1; i < count; i++) {
            int index = (newest - i + SIZE) % SIZE;
            if (laterTime - times[index] > KEEP_NANOS) break;
            Pose earlier = poses[index];
            long earlierTime = times[index];
            if (timeNanos >= earlierTime) {
                double f = laterTime == earlierTime ? 0.0 : (double) (timeNanos - earlierTime) / (laterTime - earlierTime);
                double turn = Math.atan2(Math.sin(later.heading() - earlier.heading()),
                        Math.cos(later.heading() - earlier.heading()));
                return new Pose(earlier.x() + f * (later.x() - earlier.x()),
                        earlier.y() + f * (later.y() - earlier.y()), earlier.heading() + f * turn);
            }
            later = earlier;
            laterTime = earlierTime;
        }
        return later;
    }

    public void clear() {
        count = 0;
        newest = -1;
    }
}
