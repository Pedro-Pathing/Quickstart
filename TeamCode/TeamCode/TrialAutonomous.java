package org.firstinspires.ftc.teamcode.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class TrialAutonomous extends OpMode {

    DcMotorEx shooter;
    DcMotor intake;
    CRServo leftPusher;
    CRServo rightPusher;

    Follower follower;
    Timer pathTimer;

    boolean pathStarted;
    boolean pathsBuilt = false;

    enum StateMachine {
        DRIVE_STARPOS_SHOOT_POS,
        SHOOT_TO_PREPICKUP,
        INTAKE,
        LAUNCH_PICKUP_1,
        SHOOT_TO_PREPICKUP2,
        INTAKE2,
        LAUNCH_PICKUP_2,
        SHOOT_TO_PREPICKUP3,
        INTAKE3,
        LAUNCH_PICKUP_3,
        FINISH;

    }

    StateMachine pathState;

    private Pose startPose;
    private Pose shootPose;
    private Pose preloadPose;
    private Pose intakePose;
    private Pose launchPose1;

    private Pose preloadPose2;
    private Pose intakePose2;
    private Pose launchPose2;

    private Pose preloadPose3;
    private Pose intakePose3;
    private Pose launchPose3;

    private Pose finishPose;




    private PathChain driveStartPosShootPos, shootToPrePickup, INTAKE, LAUNCH_PICKUP_1, SHOOT_TO_PREPICKUP2,
            INTAKEPOS2, LAUNCHPOSE2, PRELOADPOSE3, INTAKEPOSE3, LAUNCHPOSE3, FINISH;

    @Override
    public void init() {

        pathState = StateMachine.DRIVE_STARPOS_SHOOT_POS;

        pathTimer = new Timer();
        pathTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPusher = hardwareMap.get(CRServo.class, "leftPusher");
        rightPusher = hardwareMap.get(CRServo.class, "rightPusher");

        pathStarted = false;

        intake.setPower(0);
        leftPusher.setPower(0);
        rightPusher.setPower(0);
        shooter.setVelocity(0);
    }

    @Override
    public void init_loop() {
        if (!pathsBuilt) {
            InitializePoseValues();
        }
    }

    private void InitializePoseValues() {

        if (gamepad1.y) telemetry.addLine("Big blue selected");
        else if (gamepad1.b) telemetry.addLine("Big red selected");
        else if (gamepad1.a) telemetry.addLine("Small red selected");
        else if (gamepad1.x) telemetry.addLine("Small blue selected");
        else if (gamepad1.right_bumper && gamepad1.x) {
            telemetry.addLine("Backup selected ");

        } else {
            telemetry.addLine("Select auto mode by pressing: " +
                    "X(Small blue) / Y(Big blue) / B(Big red) / A(Small red)");
            telemetry.update();
            return;
        }

        telemetry.update();

        if (gamepad1.y) {
            startPose = new Pose(21.350906095551892, 124.7841845140033, Math.toRadians(138));
            shootPose = new Pose(47.44645799011532, 95.36738056013179, Math.toRadians(138));
            preloadPose = new Pose(47.44645799011532, 85.4036243822076, Math.toRadians(-180));
            intakePose = new Pose(26.332784184514, 91.57166392092257, Math.toRadians(-180));
            launchPose1 = new Pose(47.44645799011532, 95.36738056013179, Math.toRadians(138));
            preloadPose2 = new Pose(72.1203212890625, 65.63519836425781, Math.toRadians(-180));
            intakePose2 = new Pose(17.31795716639209, 61.680395387149915, Math.toRadians(-180));
            launchPose2 = new Pose(47.44645799011532, 95.36738056013179, Math.toRadians(132));
            preloadPose3 = new Pose(61.205761230468745, 37.861437866210935, Math.toRadians(-180));
            intakePose3 = new Pose(19.453047775947276, 43.6507413509061, Math.toRadians(-180));
            launchPose3 = new Pose(47.44645799011532, 95.36738056013179, Math.toRadians(132));
            finishPose = new Pose(35.110378912685334, 71.40691927512356, Math.toRadians(-180));
            buildPaths();
            follower.setPose(startPose);
            pathsBuilt = true;

        }

        if (gamepad1.b) {
            startPose = new Pose(
                    122.64909390444811, 124.7841845140033, Math.toRadians(40));

            shootPose = new Pose(
                    96.55354200988468, 95.36738056013179, Math.toRadians(40));

            preloadPose = new Pose(
                    96.55354200988468, 85.4036243822076, Math.toRadians(0));

            intakePose = new Pose(
                    138.16, 95.92, Math.toRadians(0));

            launchPose1 = new Pose(
                    96.55354200988468, 95.36738056013179, Math.toRadians(40));

            preloadPose2 = new Pose(
                    88.56, 62.64000000000001, Math.toRadians(0));

            intakePose2 = new Pose(
                    142.6, 68.599999999999994, Math.toRadians(0));

            launchPose2 = new Pose(
                    96.55354200988468, 95.36738056013179, Math.toRadians(40));
//10 contrl z's remove a 8 in preload p 3
            preloadPose3 = new Pose(
                    90.79423876953126, 37.861437866210935, Math.toRadians(0));

            intakePose3 = new Pose(
                    138.8, 39.6, Math.toRadians(0));

            launchPose3 = new Pose(
                    96.55354200988468, 95.36738056013179, Math.toRadians(40));

            finishPose = new Pose(
                    108.88962108731467, 71.40691927512356, Math.toRadians(0));


            buildPaths();
            follower.setPose(startPose);
            pathsBuilt = true;

        }

        if (gamepad1.a) {
            startPose = new Pose(79.90752160644531, 8.715837280273437, Math.toRadians(90));

            shootPose = new Pose(
                    96.55354200988468, 95.36738056013179, Math.toRadians(48 ));

            preloadPose = new Pose(
                    96.55354200988468, 85.4036243822076, Math.toRadians(0));

            intakePose = new Pose(
                    128.16, 85.92, Math.toRadians(0));

            launchPose1 = new Pose(
                    96.55354200988468, 95.36738056013179, Math.toRadians(48));

            preloadPose2 = new Pose(
                    88.56, 62.64000000000001, Math.toRadians(0));

            intakePose2 = new Pose(
                    129.6, 63.599999999999994, Math.toRadians(0));

            launchPose2 = new Pose(
                    96.55354200988468, 95.36738056013179, Math.toRadians(48));

            preloadPose3 = new Pose(
                    82.79423876953126, 37.861437866210935, Math.toRadians(0));

            intakePose3 = new Pose(
                    130.8, 39.6, Math.toRadians(0));

            launchPose3 = new Pose(
                    96.55354200988468, 95.36738056013179, Math.toRadians(48));

            finishPose = new Pose(
                    108.88962108731467, 71.40691927512356, Math.toRadians(0));

            buildPaths();
            follower.setPose(startPose);
            pathsBuilt = true;

        }

        if (gamepad1.x) {
            startPose = new Pose(63.87744128417969, 7.987197265625001, Math.toRadians(90));
            shootPose = new Pose(47.44645799011532, 95.36738056013179, Math.toRadians(138));
            preloadPose = new Pose(47.44645799011532, 85.4036243822076, Math.toRadians(-180));
            intakePose = new Pose(26.332784184514, 91.57166392092257, Math.toRadians(-180));
            launchPose1 = new Pose(47.44645799011532, 95.36738056013179, Math.toRadians(138));
            preloadPose2 = new Pose(64.1203212890625, 62.63519836425781, Math.toRadians(-180));
            intakePose2 = new Pose(23.960461285008236, 63.57825370675453, Math.toRadians(-180));
            launchPose2 = new Pose(47.44645799011532, 95.36738056013179, Math.toRadians(132));
            preloadPose3 = new Pose(61.205761230468745, 37.861437866210935, Math.toRadians(-180));
            intakePose3 = new Pose(20.401976935749587, 41.75288303130149, Math.toRadians(-180));
            launchPose3 = new Pose(47.44645799011532, 95.36738056013179, Math.toRadians(132));
            finishPose = new Pose(35.110378912685334, 71.40691927512356, Math.toRadians(-180));
            buildPaths();
            follower.setPose(startPose);
            pathsBuilt = true;

        }
    }

    private void buildPaths() {

        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToPrePickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preloadPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preloadPose.getHeading())
                .build();

        INTAKE = follower.pathBuilder()
                .addPath(new BezierLine(preloadPose, intakePose))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), intakePose.getHeading())
                .build();

        LAUNCH_PICKUP_1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, launchPose1))
                .setLinearHeadingInterpolation(intakePose.getHeading(), launchPose1.getHeading())
                .build();

        SHOOT_TO_PREPICKUP2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose1, preloadPose2))
                .setLinearHeadingInterpolation(launchPose1.getHeading(), preloadPose2.getHeading())
                .build();

        INTAKEPOS2 = follower.pathBuilder()
                .addPath(new BezierLine(preloadPose2, intakePose2))
                .setLinearHeadingInterpolation(preloadPose2.getHeading(), intakePose2.getHeading())
                .build();

        LAUNCHPOSE2 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, launchPose2))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), launchPose2.getHeading())
                .build();

        PRELOADPOSE3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose2, preloadPose3))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), preloadPose3.getHeading())
                .build();

        INTAKEPOSE3 = follower.pathBuilder()
                .addPath(new BezierLine(preloadPose3, intakePose3))
                .setLinearHeadingInterpolation(preloadPose3.getHeading(), intakePose3.getHeading())
                .build();

        LAUNCHPOSE3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose3, launchPose3))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), launchPose3.getHeading())
                .build();

        FINISH = follower.pathBuilder()
                .addPath(new BezierLine(launchPose3, finishPose))
                .setLinearHeadingInterpolation(launchPose3.getHeading(), finishPose.getHeading())
                .build();
    }

    private void statePathUpdate() {
        if (pathState == null)
            return;

        double shooterVelocity = -1150;
        intake.setPower(-1);


        switch (pathState) {

            case DRIVE_STARPOS_SHOOT_POS:
                startPath(driveStartPosShootPos);
                leftPusher.setPower(0);
                rightPusher.setPower(0);

                if (!follower.isBusy()) {
                    shooter.setVelocity(shooterVelocity);
                    intake.setPower(-1);
                    if (shooter.getVelocity() <= -25) {
                        if (pathTimer.getElapsedTimeSeconds() >= 1) {
                            leftPusher.setPower(1);
                        }
                        if (pathTimer.getElapsedTimeSeconds() >= 1) {   /// /////////////////////////////////////////////////////
                            shooter.setVelocity(shooterVelocity);
                            rightPusher.setPower(-1);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 8) {
                        setPathState(StateMachine.SHOOT_TO_PREPICKUP);
                    }
                }
                break;

            case SHOOT_TO_PREPICKUP:
                startPath(shootToPrePickup);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
                runIntakeWhileBusy();
                if (!follower.isBusy()) setPathState(StateMachine.INTAKE);
                break;

            case INTAKE:
                startPath(INTAKE);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
                runIntakeWhileBusy();
                if (!follower.isBusy()) setPathState(StateMachine.LAUNCH_PICKUP_1);
                break;

            case LAUNCH_PICKUP_1:
                startPath(LAUNCH_PICKUP_1);
                if (shooter.getVelocity() <= shooterVelocity && pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    leftPusher.setPower(1);
                    if (pathTimer.getElapsedTimeSeconds() >= 2.5) {
                        rightPusher.setPower(-1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 5.75) {
                        runShooterPusherWait(StateMachine.SHOOT_TO_PREPICKUP2);
                    }
                }
                break;

            case SHOOT_TO_PREPICKUP2:
                startPath(SHOOT_TO_PREPICKUP2);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
                runIntakeWhileBusy();
                if (!follower.isBusy()) setPathState(StateMachine.INTAKE2);
                break;

            case INTAKE2:
                startPath(INTAKEPOS2);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
                runIntakeWhileBusy();
                if (!follower.isBusy()) setPathState(StateMachine.LAUNCH_PICKUP_2);
                break;

            case LAUNCH_PICKUP_2:
                startPath(LAUNCHPOSE2);
                if (shooter.getVelocity() <= shooterVelocity && pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    leftPusher.setPower(1);
                    if (pathTimer.getElapsedTimeSeconds() >= 2.2) {
                        rightPusher.setPower(-1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 5.75) {
                        runShooterPusherWait(StateMachine.SHOOT_TO_PREPICKUP3);
                    }
                }
                break;

            case SHOOT_TO_PREPICKUP3:
                startPath(PRELOADPOSE3);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
                runIntakeWhileBusy();
                if (!follower.isBusy()) setPathState(StateMachine.INTAKE3);
                break;

            case INTAKE3:
                startPath(INTAKEPOSE3);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
                runIntakeWhileBusy();
                if (!follower.isBusy()) setPathState(StateMachine.LAUNCH_PICKUP_3);
                break;

            case LAUNCH_PICKUP_3:
                startPath(LAUNCHPOSE3);
                if (shooter.getVelocity() <= shooterVelocity && pathTimer.getElapsedTimeSeconds() >= 1.85) {
                    leftPusher.setPower(1);
                    if (pathTimer.getElapsedTimeSeconds() >= 2.25) {
                        rightPusher.setPower(-1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 5.75) {
                        runShooterPusherWait(StateMachine.FINISH);
                    }
                }
                break;

            case FINISH:
                if (!pathStarted) follower.followPath(FINISH, true);
                pathStarted = true;
                intake.setPower(0);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
                shooter.setVelocity(0);
                break;
        }
    }

    private void startPath(PathChain path) {
        if (!pathStarted) {
            follower.followPath(path, true);
            pathStarted = true;
            pathTimer.resetTimer();
        }
    }

    private void runIntakeWhileBusy() {
        intake.setPower(-1);
    }

    private void runShooterPusherWait(StateMachine nextState) {
        if (pathTimer.getElapsedTimeSeconds() >= 5) {
            setPathState(nextState);
        }
    }

    private void setPathState(StateMachine newState) {
        pathState = newState;
        pathStarted = false;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("STATE", pathState);

        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading", pose.getHeading());
        }

        telemetry.addData("followerBusy", follower.isBusy());
        telemetry.addData("pathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
