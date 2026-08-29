package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(group = "1")
public class AutomaticOffsetVelocityTuner extends OpMode {
    public static double POWER = 0.3;
    public static double TRIAL_RUNTIME = 8;
    private final Timer timer = new Timer();
    private boolean done = false;
    private final List<Twist> twists = new ArrayList<>();
    private Vector2D offsets;
    public static int IRLS_PASSES = 5;
    public static double OMEGA_THRESHOLD = 0.25;

    private static class Circle {
        Vector2D center;
        double radius;

        Circle(Vector2D center, double radius) {
            this.center = center;
            this.radius = radius;
        }
    }

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void start() {
        follower.setPose(Pose.zero());
        timer.reset();
        follower.manual(0, 0, POWER);
        follower.update();
    }

    /**
     * This initializes the PoseUpdater as well as the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetry.addLine("This will turn continuously in place for " + TRIAL_RUNTIME + " seconds.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.update();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated offsets and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        if (gamepad1.bWasPressed()) {
            follower.manual(0, 0, 0);
            requestOpModeStop();
        }

        if (!done) {
            if (Math.abs(follower.twist().omega()) > OMEGA_THRESHOLD) twists.add(follower.twist());
            Log.e("p", follower.pose().toString());

            if (timer.seconds() >= TRIAL_RUNTIME) {
                done = true;
                follower.manual(0, 0, 0);
                telemetry.update();
                offsets = fitVelocity(twists);
            }
        } else {
            follower.manual(0, 0, 0);
            telemetry.addLine("The following values are the offsets in inches that should be applied to your localizer.");
            telemetry.addLine("xPodOffset: " + Constants.localizerConfig.xPodOffset.get() + ", " + offsets.y());
            telemetry.addLine("yPodOffset: " + Constants.localizerConfig.yPodOffset.get() +  ", " + offsets.x());
            telemetry.update();
        }
    }

    private static Vector2D fitVelocity(List<Twist> samples) {
        int n = samples.size();
        if (n < 10) {
            return Vector2D.zero();
        }

        double[] w = new double[n];
        Arrays.fill(w, 1.0);

        double dx = 0, dy = 0;
        double[] res = new double[n];

        for (int pass = 0; pass < IRLS_PASSES; pass++) {
            Vector sum = new Vector(0, 0, 0);
            for (int i = 0; i < n; i++) {
                sum = sum.plus(samples.get(i).toVector().times(samples.get(i).omega() * w[i]));
            }

            if (sum.get(2) < 1e-9) break;
            dx = sum.get(1) / sum.get(2);
            dy = -sum.get(0) / sum.get(2);

            for (int i = 0; i < n; i++) {
                Vector2D pred = Vector2D.cartesian(dx, dy)
                        .rotate(Math.PI / 2)
                        .times(samples.get(i).omega());
                Vector2D residual = samples.get(i).toVector2D().minus(pred);
                res[i] = residual.magnitude();
            }
            double mad = medianAbs(res);
            double thresh = Math.max(mad * 1.4826, 1e-6) * 3.0;
            for (int i = 0; i < n; i++) {
                w[i] = res[i] <= thresh ? 1.0 : thresh / res[i];
            }
        }

        return Vector2D.cartesian(dx, dy);
    }

    private static double medianAbs(double[] a) {
        double[] copy = Arrays.copyOf(a, a.length);
        Arrays.sort(copy);
        int n = copy.length;
        return n % 2 == 0 ? (copy[n / 2 - 1] + copy[n / 2]) / 2.0 : copy[n / 2];
    }
}
