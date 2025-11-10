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
@Autonomous(name = "Example Auto", group = "Examples")
public class BlueFarAuto extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path scorePreload, intakeStack1, turn, scoreStack1, openGate, initialIntakeStack2, intakeStack2, reverseInitialIntakeStack2;
    private final Pose startPose = new Pose(123, 123, Math.toRadians(37));
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(37));

    private final Pose intakePose1 = new Pose(133, 84, Math.toRadians(0));

    private final Pose openGatePose = new Pose(133, 70, Math.toRadians(0));
    private final Pose openGateControlPoint = new Pose(90,76.5);
    private final Pose initialIntakePose2 = new Pose(84, 60, Math.toRadians(0));
    private final Pose intakePose2 = new Pose(133, 60, Math.toRadians(0));

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
        openGate = new Path(new BezierCurve(intakePose1, openGateControlPoint, openGatePose));
        scoreStack1 = new Path(new BezierLine(openGatePose, scorePose));
        scoreStack1.setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading());
        initialIntakeStack2 = new Path(new BezierLine(scorePose, initialIntakePose2));
        initialIntakeStack2.setLinearHeadingInterpolation(scorePose.getHeading(), initialIntakePose2.getHeading());
        intakeStack2 = new Path(new BezierLine(initialIntakePose2, intakePose2));
        intakeStack2.setLinearHeadingInterpolation(initialIntakePose2.getHeading(), intakePose2.getHeading());
        reverseInitialIntakeStack2 = new Path(new BezierLine(intakePose2, initialIntakePose2));
        reverseInitialIntakeStack2.setLinearHeadingInterpolation(intakePose2.getHeading(), initialIntakePose2.getHeading());
//

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        follower.setMaxPower(0.8);
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
                if (robot.shooter.reachCloseSpeed() || pathTimer.getElapsedTime() > 3000) {
                    robot.intake.intakeArtifacts(); // start intake to shoot
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 2000){ //TBD: change 3 secs to shorter if possible
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
                    robot.shooter.startCloseShoot();
                    setPathState(7);
                }
                break;
            case 7:
                follower.followPath(openGate);
                setPathState(8);
                break;

            case 8:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3000) {
                    follower.followPath(scoreStack1, true);
                    setPathState(9);
                }
                break;
            case 9: // shoot balls now
                if ((!follower.isBusy() || pathTimer.getElapsedTime() > 2000) && robot.shooter.reachCloseSpeed()) {
                    robot.intake.intakeArtifacts();
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTime() > 3000) {
                    robot.shooter.stopFlyWheel();
                    follower.followPath(initialIntakeStack2);
                    robot.intake.intakeArtifactsOnlyIntake();
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(intakeStack2);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(reverseInitialIntakeStack2);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(scorePreload);
                    robot.shooter.startCloseShoot();
                    setPathState(16);
                }
                break;
            case 16:
                if ((!follower.isBusy() || pathTimer.getElapsedTime() > 3000) && robot.shooter.reachCloseSpeed()) {
                    robot.intake.intakeArtifacts();
                    setPathState(-1);
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        //follower.setMaxPower(0.8);
    }
}
