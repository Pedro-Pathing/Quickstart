package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretHistory {
    public static final int SIZE = 64;

    private final ElapsedTime clock = new ElapsedTime();
    private final double[] times = new double[SIZE];
    private final double[] turretAngles = new double[SIZE];
    private final double[] xs = new double[SIZE];
    private final double[] ys = new double[SIZE];
    private final double[] headings = new double[SIZE];
    private int count = 0;
    private int next = 0;

    public static class Sample {
        public final double time;
        public final double turretAngle;
        public final double x;
        public final double y;
        public final double heading;

        public Sample(double time, double turretAngle, double x, double y, double heading) {
            this.time = time;
            this.turretAngle = turretAngle;
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public double now() {
        return clock.seconds();
    }

    public void add(double time, double turretAngle, double x, double y, double heading) {
        times[next] = time;
        turretAngles[next] = turretAngle;
        xs[next] = x;
        ys[next] = y;
        headings[next] = heading;
        next = (next + 1) % SIZE;
        count = Math.min(count + 1, SIZE);
    }

    public Sample at(double time) {
        if (count == 0) return null;
        int newest = (next - 1 + SIZE) % SIZE;
        if (time >= times[newest]) return sample(newest);

        int later = newest;
        for (int i = 1; i < count; i++) {
            int earlier = (newest - i + SIZE) % SIZE;
            if (times[earlier] <= time) {
                double t = (time - times[earlier]) / (times[later] - times[earlier]);
                return new Sample(
                        time,
                        lerp(turretAngles[earlier], turretAngles[later], t),
                        lerp(xs[earlier], xs[later], t),
                        lerp(ys[earlier], ys[later], t),
                        headings[earlier] + t * Turret.wrap(headings[later] - headings[earlier])
                );
            }
            later = earlier;
        }
        return sample(later);
    }

    private Sample sample(int i) {
        return new Sample(times[i], turretAngles[i], xs[i], ys[i], headings[i]);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
