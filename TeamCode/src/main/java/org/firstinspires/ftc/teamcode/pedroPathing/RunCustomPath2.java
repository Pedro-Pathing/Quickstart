package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Four Path Auto", group = "Auto")
public class RunCustomPath2 extends OpMode {

    private Follower follower;
    private int pathState;

    // =========================
    // POSES
    // =========================

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180));

    private final Pose scorePreloadPose = new Pose(50.674285714285716, 115.17142857142859, Math.toRadians(135));

    private final Pose grab1Pose = new Pose(70.46285714285715, 126.76000000000002, Math.toRadians(0));

    private final Pose path3Pose = new Pose(48.42857142857143, 136.72, Math.toRadians(135)); // tangential

    private final Pose path4Pose = new Pose(28.5, 128, Math.toRadians(180)); // tangential

    // =========================
    // PATHS
    // =========================

    private Path scorePreload;
    private PathChain grab1, path3, path4;

    public void buildPaths() {

        // Path 1: Start → ScorePreload
        scorePreload = new Path(new BezierLine(startPose, scorePreloadPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());

        // Path 2: ScorePreload → Grab1
        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePreloadPose, grab1Pose))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), grab1Pose.getHeading())
                .build();

        // Path 3: Grab1 → Path3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(grab1Pose, path3Pose))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), path3Pose.getHeading())
                .build();

        // Path 4: Path3 → Path4
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(path3Pose, path4Pose))
                .setLinearHeadingInterpolation(path3Pose.getHeading(), path4Pose.getHeading())
                .build();
    }

    // =========================
    // FSM
    // =========================

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grab1);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    pathState = -1; // stop
                }
                break;
        }
    }

    // =========================
    // OPMODE METHODS
    // =========================

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {}
}