package org.firstinspires.ftc.teamcode.pedroPathing.control;

import android.annotation.SuppressLint;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig;

@TeleOp(group = "3")
public class ForwardTranslationalAutoTuner extends OpMode {
    public static double BETA_LARGE = 0.6;
    public static double BETA_SMALL = 0.9;

    private static final double POWER = 0.4;
    private static final double RUNTIME = 1.2;
    private static final int SAMPLES = 15;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final Timer timer = new Timer();
    private boolean done = false;
    private double lastTime = 0.0;

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("This will run continuously in place for " + RUNTIME + " seconds.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        timer.reset();
        lastTime = timer.seconds();
        follower.manual(POWER, 0, 0);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-6;

        lastTime = now;

        follower.update();
        telemetry.update();

        telemetry.addData("done", done);
        telemetry.addData("dt", String.format("%.6f s", dt));

        if (!done) {
            times.add(timer.seconds());

            double forwardVelocity = Math.abs(follower.velocity().toVector2D().dot(Vector2D.polar(1, follower.pose().heading())));
            vMax = Math.max(vMax, forwardVelocity / POWER);

            velocities.add(forwardVelocity);
            telemetry.addData("velocity (in/s)", String.format("%.4f", velocities.get(velocities.size() - 1)));

            if (timer.seconds() >= RUNTIME) {
                done = true;
                systemIdentification();

                follower.manual(0, 0, 0);
                telemetry.addData("elapsed time (s)", String.format("%.4f", timer.seconds()));
            } else {
                follower.manual(POWER, 0, 0);
                return;
            }
        }

        double kP_large = calculatekP(BETA_LARGE);
        double kP_small = calculatekP(BETA_SMALL);

        telemetry.addData("Est tau (s)", String.format("%.4f", tau));
        telemetry.addData("Est K (in/s per power)", String.format("%.4f", K));
        telemetry.addData("Est kV", kV);
        telemetry.addData("Est kA", kA);
        telemetry.addData("Large Coefficients", "kP=" + String.format("%.4f", kP_large));
        telemetry.addData("Small Coefficients", "kP=" + String.format("%.4f", kP_small));
    }

    private double calculatekP(double beta) {
        kV = 1 / K;
        kA = tau / K * beta;
        double denominator = foresightConfig.linearBrakeCoefficients.get().get(0,0) + 2.0 * foresightConfig.quadraticBrakeCoefficients.get().get(0,0) * vMax;
        double discriminant = kA - kV * denominator;

        if (discriminant < 0) return kV * kV / (4.0 * kA);
        double sqrt = (Math.sqrt(kA) - Math.sqrt(discriminant)) / denominator;
        return sqrt * sqrt;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }

    public double[] linearFit(Double[] x, Double[] y) {
        int n = x.length;
        double sumX = 0, sumXY = 0, sumY = 0, sumX2 = 0;

        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }

        double m = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double b = (sumY - m * sumX) / n;
        return new double[] {b, m};
    }
}
