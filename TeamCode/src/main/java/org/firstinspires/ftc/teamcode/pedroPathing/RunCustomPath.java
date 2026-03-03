package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Example Auto", group = "Examples")
public class RunCustomPath extends OpMode {

    private Follower follower;

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // =========================
    // POSES
    // =========================

    private final Pose startPose   = new Pose(28.5, 128, Math.toRadians(180));
    private final Pose scorePose   = new Pose(60, 85, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    // =========================
    // PATHS
    // =========================

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;
    private PathChain grabPickup2, scorePickup2;
    private PathChain grabPickup3, scorePickup3;

    public void buildPaths() {

        // Straight line preload score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(
                startPose.getHeading(),
                scorePose.getHeading()
        );

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup1Pose.getHeading()
                )
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup1Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup2Pose.getHeading()
                )
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup2Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup3Pose.getHeading()
                )
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup3Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();
    }

    // =========================
    // FSM (STATE MACHINE)
    // =========================

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // Stop
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    // =========================
    // OPMODE METHODS
    // =========================

    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setStartingPose(startPose);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {}
}