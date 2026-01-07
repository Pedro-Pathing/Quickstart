package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Main Auto")
public class MainAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;



    
    private final Pose poseA = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose poseB = new Pose(60, 85, Math.toRadians(135));

    private final Pose poseC = new Pose(30, 105, Math.toRadians(50));


    private Path scorePreload;
    private PathChain testMove;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(poseA, poseB));
        scorePreload.setLinearHeadingInterpolation(poseA.getHeading(), poseB.getHeading());


        testMove = follower.pathBuilder()
            .addPath(new BezierLine(poseB, poseC))
            .setLinearHeadingInterpolation(poseB.getHeading(), poseC.getHeading())
            .build();
        }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(-1); // I think -1 just makes it stop
                }
                break;

        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
