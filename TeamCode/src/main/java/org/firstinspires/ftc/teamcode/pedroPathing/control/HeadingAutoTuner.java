package org.firstinspires.ftc.teamcode.pedroPathing.control;

import static com.pedropathing.utils.Utils.linearFit;

import android.annotation.SuppressLint;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Deprecated
@TeleOp(group = "3")
public class HeadingAutoTuner extends OpMode {
    private static final double ALPHA_LARGE = 0.6;
    private static final double ALPHA_SMALL = 0.9;
    private static final double BETA = 1.0;
    private static final double POWER = 0.6;
    private static final double RUNTIME = 3;
    private static final int K_SAMPLES = 15;

    private double tau;
    private double lambda_small;
    private double lambda_large;
    private double K;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> angularVelocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;
    private Follower follower;
    private int samplesUsed;
    private Mode mode = Mode.CALIBRATION;
    private String largeCoefficients;
    private String smallCoefficients;
    private String headingFeedforward;

    private enum Mode {
        CALIBRATION,
        HEADING_LOCK
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("This will turn continuously in place for " + RUNTIME + " seconds.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        follower.setPose(Pose.zero());
        timer.reset();
        lastTime = timer.seconds();
        follower.manual(0, 0, POWER);
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

        telemetry.addData("done", done);
        telemetry.addData("dt", String.format("%.6f s", dt));

        if (mode.equals(Mode.CALIBRATION)) {
            if (!done) {
                times.add(timer.seconds());
                angularVelocities.add(Math.abs(follower.velocity().omega));
                telemetry.addData("angular velocity (rad/s)", String.format("%.4f", angularVelocities.get(angularVelocities.size() - 1)));

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    follower.manual(0, 0, 0);
                    telemetry.addData("elapsed time (s)", String.format("%.4f", timer.seconds()));
                } else {
                    follower.manual(0, 0, POWER);
                    telemetry.update();
                    return;
                }

                telemetry.update();
            }

            lambda_small = tau * ALPHA_SMALL;
            lambda_large = tau * ALPHA_LARGE;

            double kDLarge = getkD(lambda_large);
            double kPLarge = getkP(lambda_large);
            double kDSmall = getkD(lambda_small);
            double kPSmall = getkP(lambda_small);

            double feedforward = BETA / K;

            if (gamepad1.aWasPressed() && done) {
                mode = Mode.HEADING_LOCK;
                follower.hold(new Pose(72, 72, 0));
                Constants.foresightConfig.headingFeedback.set(Controller.piecewise(
                        Controller.pid(kPSmall, 0, kDSmall)
                ).put(
                        Math.PI / 20,
                        Controller.pid(kPLarge, 0, kDLarge)
                ));
                largeCoefficients = "kP=" + String.format("%.4f", kPLarge) + ", kD=" + String.format("%.4f", kDLarge);
                smallCoefficients = "kP=" + String.format("%.4f", kPSmall) + ", kD=" + String.format("%.4f", kDSmall);
                headingFeedforward = "k=" + String.format("%.4f", feedforward);
            }

            telemetry.addData("samples used", samplesUsed);
            telemetry.addData("Large Coefficients", "kP=" + String.format("%.4f", kPLarge) + ", kD=" + String.format("%.4f", kDLarge));
            telemetry.addData("Small Coefficients", "kP=" + String.format("%.4f", kPSmall) + ", kD=" + String.format("%.4f", kDSmall));
            telemetry.addData("Heading Feedforward", "k=" + String.format("%.4f", feedforward));
            telemetry.addLine();
            telemetry.addData("Est tau (s)", String.format("%.4f", tau));
            telemetry.addData("Est K (rad/s per power)", String.format("%.4f", K));
            telemetry.addData("Lambda large (s)", String.format("%.4f", lambda_large));
            telemetry.addData("Lambda small (s)", String.format("%.4f", lambda_small));
        } else {
            telemetry.addLine("Holding heading with updated values");
            telemetry.addData("Large Coefficients", largeCoefficients);
            telemetry.addData("Small Coefficients", smallCoefficients);
            telemetry.addData("Heading Feedforward", headingFeedforward);
            telemetry.addData("Heading Error", -follower.pose().heading());
            telemetry.addData("Heading Velocity", follower.velocity().omega);
        }

        telemetry.update();
    }

    private double getkP(double lambda) {
        return tau / (K * lambda * lambda);
    }

    private double getkD(double lambda) {
        return 1 / K * (2 * tau / lambda - 1);
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - K_SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += angularVelocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = angularVelocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        samplesUsed = x.size();
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }
}
