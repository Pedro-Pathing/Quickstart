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
    private Path getFirstPattern, shootStack1;
    private final Pose startPose = new Pose(441, 8, Math.toRadians(0));
    private final Pose firstPattern = new Pose(133, 84, Math.toRadians(0));
    private final Pose secondPattern = new Pose(133, 60, Math.toRadians(0));
    private final Pose thirdPattern = new Pose(133, 36, Math.toRadians(0));
    private final Pose shootingPose = new Pose(56, 12, Math.toRadians(0));

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
        getFirstPattern = new Path(new BezierLine(startPose, thirdPattern));
        shootStack1 = new Path(new BezierLine(thirdPattern, shootingPose));
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
                if (robot.shooter.reachFarSpeed() || pathTimer.getElapsedTime() > 4000) {
                    robot.intake.intakeArtifacts();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 2000){ //TBD: change 3 secs to shorter if possible
                    robot.shooter.stopFlyWheel();
                    robot.intake.intakeArtifactsOnlyIntake();
                    follower.setMaxPower(0.25);
                    follower.followPath(getFirstPattern, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000)  {//TBD: change 2 secs to shorter if possible
                    follower.followPath(shootStack1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {//TBD: change 2 secs to shorter if possible
                    follower.setMaxPower(0.8);
                    robot.shooter.startFarShoot();
                    setPathState(-1);
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
