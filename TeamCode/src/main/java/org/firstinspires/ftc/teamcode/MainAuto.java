
package org.firstinspires.ftc.teamcode; // make sure this aligns with class location
//HIIII pls don't delete

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "Main Auto")
public class MainAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;



    /*
    private final Pose poseA = new Pose(48, 12, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose poseB = new Pose(18, 84, Math.toRadians(135));
    private final Pose poseC = new Pose(74, 77, Math.toRadians(50));
    private final Pose poseD = new Pose(21, 60, Math.toRadians(50));
    private final Pose poseE = new Pose(73, 80, Math.toRadians(50));
    private final Pose poseF = new Pose(18, 36, Math.toRadians(50));
    private final Pose poseG = new Pose(75, 81, Math.toRadians(50));

    private Path startPosition;
    private PathChain moveB, moveC, moveD, moveE, moveF;
    */

    private final Pose startPose = new Pose(48, 12, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose pickUp1 = new Pose(18, 84, Math.toRadians(180));

    private final Pose pickUp1C1 = new Pose(57, 90);
    private final Pose pickUp1C2 = new Pose(57, 84);

    private final Pose pickUp2 = new Pose(18, 60, Math.toRadians(180));

    private final Pose pickUp2C1 = new Pose(75, 57);
    private final Pose pickUp2C2 = new Pose(66, 60);

    private final Pose pickUp3 = new Pose(18, 36, Math.toRadians(180));

    private final Pose pickUp3C1 = new Pose(72, 30);
    private final Pose pickUp3C2 = new Pose(72, 36);

    private final Pose shootPose = new Pose(75, 87, Math.toRadians(330));


    private Path startPosition;
    private PathChain shoot1, intake2, shoot2, intake3, shoot3, goToStart;
    public void buildPaths() {
        //startPosition = new Path(new BezierLine(startPose, pickUp1));
        startPosition = new Path(new BezierCurve(startPose, pickUp1C1, pickUp1C2, pickUp1));
        startPosition.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));


        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUp1, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(330))
                .build();

        intake2 = follower.pathBuilder()
                //.addPath(new BezierLine(shootPose, pickUp2))
                .addPath(new BezierCurve(shootPose, pickUp2C1, pickUp2C2 ,pickUp2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(pickUp2, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(330))
                .build();

        intake3 = follower.pathBuilder()
                //.addPath(new BezierLine(shootPose, pickUp3))
                .addPath(new BezierCurve(shootPose, pickUp3C1, pickUp3C2 ,pickUp3))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(pickUp3, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(330))
                .build();

        goToStart = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();



    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(startPosition);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(shoot1, 0.7, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intake2, .7,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(shoot2, .7,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intake3, .7,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(shoot3, .7,true);
                    setPathState(6);

                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToStart, .7,true);
                    setPathState(-1);

                }
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }



    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
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
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


}
