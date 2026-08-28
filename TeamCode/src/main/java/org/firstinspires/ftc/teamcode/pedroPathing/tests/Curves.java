package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import static com.pedropathing.api.Paths.curve;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(group = "4")
public class Curves extends OpMode {
    public static double DISTANCE = 48;
    public double loops = 0, lastLoop = 0, loopTime = 0;
    private Path forwards, backwards;
    private boolean forward;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
    }

    @Override
    public void start() {
        forwards = curve(new Pose(72,72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72)).
                heading(new Interpolator() {
                    @Override
                    public double interpolate(Curve curve, double t) {
                        return Math.PI;
                    }
                });
        backwards = curve(new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(72,72))
                .heading(Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1.0, Interpolator.constant(0)));
        follower.follow(forwards);
    }

    @Override
    public void loop() {
        loops++;

        if (loops > 10) {
            double now = System.currentTimeMillis();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        double nanoBefore = System.nanoTime();

        follower.update();

        telemetry.addData("Calculation Nano Time", System.nanoTime() - nanoBefore);
        telemetry.addData("Calculation Ms", 1e-6 * (System.nanoTime() - nanoBefore));

        if (follower.atParametricEnd()) {
            if (forward) {
                follower.follow(backwards);
            } else {
                follower.follow(forwards);
            }
            forward = !forward;
        }

        telemetry.addData("Loop Time Hz", 1000/loopTime);
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Following?", follower.following());
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }
}
