package org.firstinspires.ftc.teamcode.behaviorControl;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoShoot2", group = "Autonomous")
public class AutoShoot2 extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses from your updated AutoPath
    private final Pose start = poseFactory.of(58.1199, 8.0554, 270);
    private final Pose path1 = poseFactory.of(5.7595, 103.6232, 360);

    // Tunable values
    private static final double LAUNCHER_VELOCITY = 1120;
    private static final long SPIN_UP_MS = 4000;
    private static final long FEED_MS = 3000;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(40, 0, 0, 12.5);
        launcher.setVelocity(0);
        windmillServo.setPower(0);

        waitForStart();

        // 1. Spin up the launcher
        launcher.setVelocity(LAUNCHER_VELOCITY);
        waitMs(SPIN_UP_MS);

        // 2. Feed the balls
        windmillServo.setPower(1);
        waitMs(FEED_MS);

        // 3. Stop the shooter
        launcher.setVelocity(0);
        windmillServo.setPower(0);

        // 4. Drive the path to park
        follower.follow(path1());

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            showTelemetry();
        }

        launcher.setVelocity(0);
        windmillServo.setPower(0);
    }

    // Waits while still updating the follower so the robot holds its position
    private void waitMs(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            follower.update();
            showTelemetry();
        }
    }

    private void showTelemetry() {
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.update();
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }
}