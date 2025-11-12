//package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.Robot;
//@Autonomous(name = "Red Wall Close Auto", group = "Examples")
//public class RedWallCloseAutoWithLL extends OpMode {
//    private Robot robot;
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//    private int pathState;
//    private Path scorePreload, intakeStack1, turn, scoreStack1, openGate, initialIntakeStack2, intakeStack2, reverseInitialIntakeStack2, scoreStack2, intakeStack3, scoreStack3;
//    private final Pose startPose = new Pose(88, 135, Math.toRadians(270));
//    private final Pose intakePose1Control1 = new Pose(88, 72);
//    private final Pose intakePose1Contol2 = new Pose(77,85);
//    private final Pose scorePose = new Pose(84, 84, Math.toRadians(37));
//
//    private final Pose intakePose1 = new Pose(133, 84, Math.toRadians(0));
//
//    private final Pose openGatePose = new Pose(133, 70, Math.toRadians(0));
//    private final Pose openGateControlPoint = new Pose(90,76.5);
//    private final Pose initialIntakePose2 = new Pose(84, 60, Math.toRadians(0));
//    private final Pose intakePose2Control1 = new Pose(90, 46);
//    private final Pose intakePose2Control2 = new Pose(77, 62);
//    private final Pose intakePose2 = new Pose(133, 60, Math.toRadians(0));
//    private final Pose intakePose3Control1 = new Pose(88, 19);
//    private final Pose intakePose3Control2 = new Pose(77, 38);
//    private final Pose intakePose3 = new Pose(133, 36, Math.toRadians(0));
//
//    private double turretClosePosition = 0.25; // changed to double
//
//    @Override
//    public void init() {
//        // Timers
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//
//        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
//        telemetry.update();
//        robot = new Robot(hardwareMap);
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//        openGate = new Path(new BezierCurve(intakePose1, openGateControlPoint, openGatePose));
//        intakeStack1 = new Path(new BezierCurve(startPose, intakePose1Control1, intakePose1Contol2, intakePose1));
//        scoreStack1 = new Path(new BezierLine(intakePose1, scorePose));
//        scoreStack1.setLinearHeadingInterpolation(intakePose1.getHeading(), scorePose.getHeading());
//        intakeStack2 = new Path(new BezierCurve(scorePose, intakePose2Control1, intakePose2Control2, intakePose2));
//        intakeStack2.setLinearHeadingInterpolation(scorePose.getHeading(), intakePose2.getHeading());
//        scoreStack2 = new Path(new BezierLine(intakePose2, scorePose));
//        scoreStack2.setLinearHeadingInterpolation(intakePose2.getHeading(), scorePose.getHeading());
//        intakeStack3 = new Path(new BezierCurve(scorePose, intakePose3Control1, intakePose3Control2, intakePose3));
//        intakeStack3.setLinearHeadingInterpolation(scorePose.getHeading(), intakePose3.getHeading());
//        scoreStack3 = new Path(new BezierLine(intakePose3, scorePose));
//        scoreStack2.setLinearHeadingInterpolation(intakePose3.getHeading(), scorePose.getHeading());
////
//
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        follower.setMaxPower(0.8);
//        setPathState(0);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("path timer", pathTimer.getElapsedTime());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                robot.shooter.startAutoCloseShoot(); // start shooter for close shots
//                setPathState(1);
//                break;
//            case 1:
//                if (robot.shooter.reachCloseSpeed() || pathTimer.getElapsedTime() > 5000) {
//                    robot.intake.startIntakeAndTransfer(); // start intake to shoot
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (pathTimer.getElapsedTime()>2000) {
//                    robot.shooter.stopShoot();
//                    robot.intake.stopTransfer();
//                    robot.intake.startIntakeOnly();
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (pathTimer.getElapsedTime()>2000) {
//                    follower.followPath(intakeStack1);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000){
//                    robot.shooter.startAutoCloseShoot();
//                    follower.followPath(scoreStack1);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000){
//                    robot.intake.startIntakeAndTransfer(); //Shoot to score
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if (pathTimer.getElapsedTime()>2000) {
//                    robot.shooter.stopShoot();
//                    robot.intake.stopTransfer();
//                    robot.intake.startIntakeOnly();
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (pathTimer.getElapsedTime()>2000){
//                    follower.followPath(intakeStack2);
//                    robot.shooter.startCloseShoot();
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000 && robot.shooter.reachCloseSpeed()){
//                    follower.followPath(scoreStack2);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000){
//                    robot.intake.startIntakeAndTransfer(); //shoot to score
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if (pathTimer.getElapsedTime()>2000) {
//                    robot.shooter.stopShoot();
//                    robot.intake.stopTransfer();
//                    robot.intake.startIntakeOnly();
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if (pathTimer.getElapsedTime()>2000){
//                    follower.followPath(intakeStack3);
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000){
//                    robot.shooter.startCloseShoot();
//                    follower.followPath(scoreStack3);
//                    setPathState(13);
//                }
//                break;
//            case 13:
//                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000 && robot.shooter.reachCloseSpeed()){
//                    robot.intake.startIntakeAndTransfer();
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                if (pathTimer.getElapsedTime()>4000){
//                    robot.shooter.stopShoot();
//                    robot.intake.stopIntake();
//                    robot.intake.stopTransfer();
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}
