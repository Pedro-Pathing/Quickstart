package org.firstinspires.ftc.teamcode.LOADCode.Autos; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestAutoR1", group = "TestAuto")
public class TestAutoR1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private boolean shooting;

    private final Pose startPose = new Pose(87, 8.8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(86, 22, Math.toRadians(80)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickupR3PoseA = new Pose(110.4, 83.6, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupR3PoseB = new Pose(121.9, 83.6, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupR2PoseA = new Pose(110.4, 59.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickupR2PoseB = new Pose(121.9, 59.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickupR1PoseA = new Pose(110.4, 35.5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickupR1PoseB = new Pose(121.9, 35.5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private PathChain grabPickup1, scorePickup, grabPickup2, grabPickup3;

    public void buildPaths() {

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        new Pose(91.100, 35.700),
                        pickupR1PoseA
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickupR1PoseA.getHeading())
                .addPath(new BezierLine(
                        pickupR1PoseA,
                        pickupR1PoseB
                ))
                .setLinearHeadingInterpolation(pickupR1PoseA.getHeading(), pickupR1PoseA.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(91.100, 60.5),
                        pickupR2PoseA
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupR2PoseA.getHeading())
                .addPath(new BezierLine(
                        pickupR2PoseA,
                        pickupR2PoseB
                ))
                .setLinearHeadingInterpolation(pickupR2PoseA.getHeading(), pickupR2PoseA.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(91.100, 84.6),
                        pickupR3PoseA
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupR3PoseA.getHeading())
                .addPath(new BezierLine(
                        pickupR3PoseA,
                        pickupR3PoseB
                ))
                .setLinearHeadingInterpolation(pickupR3PoseA.getHeading(), pickupR3PoseA.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        scorePose
                ))
                .setLinearHeadingInterpolation(follower.getHeading(), scorePose.getHeading())
                .setVelocityConstraint(0.5)
                .build();
    }

    public void delay(int time){
        Timer wait = new Timer();
        wait.resetTimer();
        while (wait.getElapsedTime() < time){}
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                //Shoot balls

                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup,true);
                    setPathState(3);
                }
                break;
            case 3:

                //Shoot balls

                setPathState(4);
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup,true);
                    setPathState(6);
                }
                break;
            case 6:

                //Shoot balls

                setPathState(7);
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup, true);
                    setPathState(9);
                }
                break;
            case 9:

                //Shoot balls

                setPathState(10);
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shooting = false;


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("shooting", shooting);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}