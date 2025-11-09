package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.Robot;
@Autonomous(name = "Example Auto", group = "Examples")
public class BlueFarAuto extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path scorePreload, intakeStack1, turn;
    private final Pose startPose = new Pose(123, 123, Math.toRadians(37));
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(37));

    private final Pose intakePose1 = new Pose(133, 84, Math.toRadians(0));
    private double turretClosePosition = 0.25; // changed to double

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
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        turn = new Path(new BezierLine(scorePose, scorePose));
        turn.setLinearHeadingInterpolation(scorePose.getHeading(), intakePose1.getHeading());
        intakeStack1 = new Path(new BezierLine(scorePose, intakePose1));
        intakeStack1.setLinearHeadingInterpolation(intakePose1.getHeading(), intakePose1.getHeading());
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
                robot.shooter.startCloseShoot(); // start shooter for close shots
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                  setPathState(2);
                }
                break;
            case 2:
                if (robot.shooter.reachCloseSpeed() || pathTimer.getElapsedTime() > 1000) {
                    robot.intake.intakeArtifacts(); // start intake to shoot
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 1000){ //TBD: change 3 secs to shorter if possible
                    robot.shooter.stopFlyWheel();
                    robot.intake.intakeArtifactsOnlyIntake();
                    follower.setMaxPower(0.25);
                    follower.followPath(turn, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000)  {//TBD: change 2 secs to shorter if possible
                    follower.followPath(intakeStack1, true);
                    setPathState(6);
                    }
                break;
            case 6:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {//TBD: change 2 secs to shorter if possible
                    follower.setMaxPower(0.8);
                    setPathState(7);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        follower.setMaxPower(0.8);
    }
}
