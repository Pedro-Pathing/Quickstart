package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@TeleOp(group = "4")
public class StrafeLineTest extends OpMode {
    public static double DISTANCE = 48;
    public double loops = 0, lastLoop = 0, loopTime = 0;
    private Path line1, line2;
    private boolean forward;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
    }

    @Override
    public void start() {
        line1 = line(new Pose(72,72, 0), new Pose(DISTANCE + 72,72, 0)).constant(0);
        line2 = line(new Pose(DISTANCE + 72,72, 0), new Pose(72,72, 0)).constant(0);
        follower.follow(line1);
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

        if (follower.distanceToEndpoint() < 5) {
            if (forward) {
                follower.follow(line2);
            } else {
                follower.follow(line1);
            }
            forward = !forward;
        }

        telemetry.addData("velocity", follower.velocity().toVector2D().x());
        telemetry.addData("Loop Time Hz", 1000/loopTime);
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Following?", follower.following());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Parametric End", follower.atParametricEnd());
        telemetry.addData("Distance", follower.distanceToEndpoint());
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("forward", forward);
        telemetry.addData("power", Arrays.toString(((Mecanum) follower.drivetrain).wheelPowers));
        telemetry.update();
    }
}
