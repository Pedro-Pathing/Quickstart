package org.firstinspires.ftc.teamcode.pedroPathing.control;

import static com.pedropathing.utils.Utils.linearFit;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig;

import android.annotation.SuppressLint;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "3")
public class ForwardTranslationalAutoTuner extends OpMode {
    public static double ALPHA_LARGE = 7.6;
    public static double ALPHA_SMALL = 4.4;
    public static double VEL_AGGRESSIVENESS = 0.85;

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
    private final ElapsedTime timer = new ElapsedTime();
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
        follower.setPose(Pose.zero());
        timer.reset();
        lastTime = timer.seconds();
        follower.manual(POWER, 0, 0);
        follower.update();
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

            double forwardVelocity = Math.abs(follower.twist().toVector2D().x());
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

        double kP_large = calculatekP(ALPHA_LARGE);
        double kP_small = calculatekP(ALPHA_SMALL);

        telemetry.addData("Est tau (s)", String.format("%.4f", tau));
        telemetry.addData("Est K (in/s per power)", String.format("%.4f", K));
        telemetry.addData("Est kV", kV);
        telemetry.addData("Est kA", kA);
        telemetry.addData("Primary Forward Translational", "kP=" + String.format("%.4f", kP_large));
        telemetry.addData("Secondary Forward Translational", "kP=" + String.format("%.4f", kP_small));
        telemetry.addData("Drive Feedforward", "kV=" + String.format("%.4f", kV * VEL_AGGRESSIVENESS));
    }

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
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
}
