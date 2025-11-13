package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
@Autonomous(name = "Red Far Auto", group = "Examples")
public class RedFarAuto extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path getFirstPattern, shootStack1, getSecondPattern, shootStack2, getThirdPattern, shootStack3;
    private final Pose startPose = new Pose(73.5, 13, Math.toRadians(0));
    private final Pose firstPattern = new Pose(129, 93, Math.toRadians(0));
    private final Pose controlPoint5 = new Pose(87, 116, Math.toRadians(0));
    private final Pose controlPoint6 = new Pose(68.5, 85, Math.toRadians(0));
    private final Pose secondPattern = new Pose(135, 68.5, Math.toRadians(0));
    private final Pose controlPoint3 = new Pose(80.7, 76, Math.toRadians(0));
    private final Pose controlPoint4 = new Pose(65.7, 67, Math.toRadians(0));
    private final Pose thirdPattern = new Pose(132.5, 44, Math.toRadians(0));
    private final Pose controlPoint1 = new Pose(83, 55, Math.toRadians(0));
    private final Pose controlPoint2 = new Pose(69,43,Math.toRadians(0));
    private final Pose shootingPose = new Pose(72.5, 14, Math.toRadians(0));

    @Override
    public void init() {
        // Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
        robot = new Robot(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        getThirdPattern = new Path(new BezierCurve(startPose, controlPoint1, controlPoint2, thirdPattern));
        getThirdPattern.setConstantHeadingInterpolation(startPose.getHeading());
        shootStack1 = new Path(new BezierLine(thirdPattern, shootingPose));
        shootStack1.setConstantHeadingInterpolation(thirdPattern.getHeading());
        getSecondPattern = new Path(new BezierCurve(shootingPose, controlPoint3, controlPoint4, secondPattern));
        getSecondPattern.setConstantHeadingInterpolation(shootingPose.getHeading());
        shootStack2 = new Path(new BezierLine(secondPattern, shootingPose));
        shootStack2.setConstantHeadingInterpolation(secondPattern.getHeading());
        getFirstPattern = new Path(new BezierCurve(shootingPose, controlPoint5, controlPoint6, firstPattern));
        getFirstPattern.setConstantHeadingInterpolation(shootingPose.getHeading());
        shootStack3 = new Path(new BezierCurve(firstPattern, controlPoint6,controlPoint5, shootingPose));
        shootStack3.setConstantHeadingInterpolation(firstPattern.getHeading());
//

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("path timer", pathTimer.getElapsedTime());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.shooter.startFarShoot();
                setPathState(1);
                break;
            case 1:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 4000) {
                    robot.intake.shootArtifacts();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 2000){
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopTransfer();
                    follower.setMaxPower(0.25);
                    follower.followPath(getThirdPattern, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000)  {
                    follower.followPath(shootStack1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    follower.setMaxPower(0.8);
                    robot.shooter.startFarShoot();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 4000) {
                    robot.intake.shootArtifacts();
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 2000){
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopTransfer();
                    follower.setMaxPower(0.25);
                    follower.followPath(getSecondPattern, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000)  {
                    follower.followPath(shootStack2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    follower.setMaxPower(0.8);
                    robot.shooter.startFarShoot();
                    setPathState(9);
                }
            case 9:
                if (robot.shooter.reachedSpeed()) {
                    robot.intake.shootArtifacts();
                    setPathState(10);
                }
            case 10:
                if (pathTimer.getElapsedTime() > 4000){
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopTransfer();
                    follower.setMaxPower(0.25);
                    follower.followPath(getFirstPattern, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy())  {
                    follower.followPath(shootStack3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    follower.setMaxPower(0.8);
                    robot.shooter.startFarShoot();
                    setPathState(13);
                }
            case 13:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 4000) {
                    robot.intake.shootArtifacts();
                    setPathState(-1);
                }





        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        follower.setMaxPower(0.8);
    }
}