package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "DecodeAuto", group = "Autonomous")
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose loadPose = new Pose(10, 9, Math.toRadians(180)); // blue loading zone
    private final Pose scorePose = new Pose(64, 80, Math.toRadians(140)); // where we shoot
    private final Pose pickup1Pose = new Pose(18.24, 84.85, Math.toRadians(180)); // row of balls closest to goal
    private final Pose pickup2Pose = new Pose(18.24, 60, Math.toRadians(180)); // middle row
    private final Pose pickup3Pose = new Pose(18.24, 35, Math.toRadians(180)); // row closest to loading zone

    private Path start_path;
    private PathChain load_path, pickup_path1, pickup_path2;

    public void createPaths() {

        load_path = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(72, 30), loadPose)) // moves to loading zone from scoring position
                .setLinearHeadingInterpolation(scorePose.getHeading(), loadPose.getHeading())
                .addPath(new BezierCurve(follower.getPose(), new Pose(72, 30), scorePose)) // moves back to scoring position
                .setLinearHeadingInterpolation(loadPose.getHeading(), scorePose.getHeading())
                .build();

        pickup_path1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,  pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        pickup_path2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(62.69, 58), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierCurve(pickup2Pose, new Pose(62.69, 58), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        pickup_path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,  pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    private boolean pickedup1, pickedup2 = false;

    public void PathUpdate() { // state machine
        switch (pathState) {
            case 0:
                follower.followPath(load_path);
                if (!pickedup1) {
                    changePath(1);
                }
                break;
            case 1:
                follower.followPath(pickup_path1);
                pickedup1 = true;
                changePath(2);
                break;
            case 2:
                follower.followPath(pickup_path2);
                pickedup2 = true;
                break;

        }
    }

    public void changePath(int change) {
        pathState = change;
        pathTimer.resetTimer();
    }

    // Called once when we hit INIT
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        createPaths();
        follower.setStartingPose(scorePose);
    }

    // Called once when we hit PLAY
    public void start() {
        opmodeTimer.resetTimer();
        changePath(0);
    }

    // LOOPS as the op mode is running
    public void loop() {
        follower.update();
        PathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time left", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();

        if (opmodeTimer.getElapsedTimeSeconds() >= 30) {
            return; // perhaps not necessary
        }
    }
}
