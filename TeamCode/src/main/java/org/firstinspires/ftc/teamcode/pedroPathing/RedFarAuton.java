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
public class RedFarAuton extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path scorePreload, intakeStack1, turn, scoreStack1, openGate, shootArtifactsFar;
    private final Pose startPose = new Pose(8, 56, Math.toRadians(90));
    private final Pose scorePose = new Pose(8, 135, Math.toRadians(90));
    private final Pose intakePose1 = new Pose(12, 56, Math.toRadians(90));

    @Override
    public void init() {
        // Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
        robot = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        intakeStack1 = new Path(new BezierLine(startPose, scorePose));
        intakeStack1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        turn = new Path(new BezierLine(scorePose, scorePose));
        turn.setLinearHeadingInterpolation(scorePose.getHeading(), intakePose1.getHeading());
        shootArtifactsFar = new Path(new BezierLine(scorePose, intakePose1));
        shootArtifactsFar.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
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
                robot.shooter.startFarShoot(); // start shooter for close shots
                setPathState(1);
                break;
            case 1:
                if (robot.shooter.reachFarSpeed()) {
                    robot.intake.intakeArtifacts();
//                    if(pathTimer.getElapsedTime() > 2000) {
//                        robot.intake.trans
//                        robot.intake.intakeArtifactsOnlyIntake();
//                        if(pathTimer.getElapsedTime(4000)) {
//                            robot.intake.transferOnly();
//                        }
//                        if(pathTimer.getElapsedTime(6000))
//
//                    }
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 3000) {
                    robot.shooter.stopFlyWheel();
                    robot.intake.intakeStop();
                    robot.shooter.startReverseShoot();
                    follower.followPath(intakeStack1, true);
                    setPathState(4);
                }
                break;
//            case 3:
//                if (pathTimer.getElapsedTime() >10000){ //TBD: change 3 secs to shorter if possible
//
//                    setPathState(4);
//                }
//                break;
//            case 3:
//                if(!follower.isBusy() || pathTimer.getElapsedTime() > 5000)  {//TBD: change 2 secs to shorter if possible
//                    follower.followPath(intakeStack1, true);
//                    setPathState(4);
//                }
//                break;
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 7000) {//TBD: change 2 secs to shorter if possible
                    follower.followPath(shootArtifactsFar, true);
                    robot.shooter.startFarShoot();
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 1000) {
                    robot.intake.intakeArtifacts();
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