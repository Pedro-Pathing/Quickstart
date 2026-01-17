package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.friends.HardwareMap;

@Autonomous(name = "Pedro Multi-Ball")
public class AutoMechanism extends LinearOpMode {

    // ===== Robot Systems =====
    HardwareMap robot;
    Follower follower;

    // ===== State Machine =====
    enum AutoState {
        DRIVE_TO_INTAKE,
        DRIVE_TO_SHOOT,
        SPIN_UP_SHOOTER,
        FEED_BALL,
        DONE
    }
    AutoState currentState = AutoState.DRIVE_TO_INTAKE;

    // ===== Timing & Counters =====
    ElapsedTime stateTimer = new ElapsedTime();
    int ballsShot = 0;

    // ===== Shooter Constants =====
    static final double TARGET_RPM = 3200;
    static final double RPM_TOLERANCE = 100;
    static final double FEED_TIME = 0.4; // seconds per ball

    @Override
    public void runOpMode() {

        // ===== INIT =====
        robot = new HardwareMap(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // ===== POSES =====
        Pose startPose  = new Pose(0.0, 0.0, 0.0);
        Pose intakeEnd  = new Pose(24.0, 0.0, 0.0); // drive straight line while intaking
        Pose shootPose  = new Pose(48.0, 12.0, Math.toRadians(90));

        follower.setPose(startPose);

        // ===== PATHS =====
        PathChain intakePath = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(startPose, intakeEnd)))
                .build();

        PathChain toShoot = new PathBuilder(follower)
                .addPath(new Path(new BezierCurve(intakeEnd, shootPose)))
                .build();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        // ===== START AUTO =====
        robot.startIntake();           // start intake immediately
        follower.followPath(intakePath); // drive straight while intaking

        while (opModeIsActive()) {
            follower.update();

            switch(currentState) {

                case DRIVE_TO_INTAKE:
                    if (!follower.isBusy()) {  // finished intake line
                        robot.stopIntake();
                        follower.followPath(toShoot); // drive to shooting position
                        robot.setShooterRPM(TARGET_RPM); // spin up shooter while driving
                        currentState = AutoState.DRIVE_TO_SHOOT;
                    }
                    break;

                case DRIVE_TO_SHOOT:
                    if (!follower.isBusy()) { // arrived at shooting position
                        stateTimer.reset();
                        currentState = AutoState.SPIN_UP_SHOOTER;
                    }
                    break;

                case SPIN_UP_SHOOTER:
                    if (robot.shooterAtSpeed(RPM_TOLERANCE)) {
                        robot.feedBall();
                        stateTimer.reset();
                        currentState = AutoState.FEED_BALL;
                    }
                    break;

                case FEED_BALL:
                    if (stateTimer.seconds() > FEED_TIME) {
                        robot.resetFeeder();
                        ballsShot++;

                        if (ballsShot < 3) { // number of balls to shoot
                            stateTimer.reset();
                            currentState = AutoState.SPIN_UP_SHOOTER;
                        } else {
                            robot.stopShooter();
                            currentState = AutoState.DONE;
                        }
                    }
                    break;

                case DONE:
                    //  add parking code here
                    break;
            }

            // ===== TELEMETRY =====
            telemetry.addData("State", currentState);
            telemetry.addData("Balls Shot", ballsShot);
            telemetry.addData("Shooter RPM", robot.getShooterRPM());
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }
    }
}