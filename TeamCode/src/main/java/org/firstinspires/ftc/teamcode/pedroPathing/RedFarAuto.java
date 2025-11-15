package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;

@Autonomous(name = "Red Far Auto", group = "Competition", preselectTeleOp="RobotTeleop")
public class RedFarAuto extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path goToFirstPattern, shootStack1, goToSecondPattern, shootStack2, goToThirdPattern, shootStack3, getFirstPattern, getSecondPattern, getThirdPattern, endingAuton;
    private final Pose startPose = new Pose(98, 8, Math.toRadians(0));
    private final Pose firstPattern = new Pose(131, 83.4, Math.toRadians(0));
    private final Pose firstPatternPickUp = new Pose(144, 83.5, Math.toRadians(0));
    private final Pose controlPoint5 = new Pose(89, 89);
    private final Pose controlPoint6 = new Pose(66.5, 84.7);
    private final Pose secondPattern = new Pose(98, 59.5, Math.toRadians(0));
    private final Pose secondPatternPickUp = new Pose(144, 59.5, Math.toRadians(0));
    private final Pose controlPoint3 = new Pose(80.4, 62.5);
    private final Pose controlPoint4 = new Pose(65.9, 60.1);
    private final Pose thirdPattern = new Pose(98, 36, Math.toRadians(0));
    private final Pose thirdPatternPickUp = new Pose(144, 36, Math.toRadians(0));
    private final Pose controlPoint1 = new Pose(77.8, 44);
    private final Pose controlPoint2 = new Pose(71.9,32.3);
    private final Pose shootingPose = new Pose(98, 8, Math.toRadians(0));
    private final Pose finalPose = new Pose(120, 10, Math.toRadians(0));
    private CRServo turretCR;

    // Turret PID constants - TUNED for smooth tracking
    private static final double TURRET_MANUAL_POWER = 0.45;
    private static final double TURRET_KP = 0.045;  // Proportional - reduced for less aggression
    private static final double TURRET_KI = 0.002;  // Integral - for steady-state accuracy
    private static final double TURRET_KD = 0.015;  // Derivative - dampens oscillation
    private static final double TURRET_DEADZONE = 0.3; // Tighter alignment threshold
    private static final double TURRET_MAX_POWER = 0.7; // Increased max for fast response
    private static final double TURRET_MIN_POWER = 0.05; // Minimum power to overcome friction

    // Velocity limiting - prevents servo from quitting on fast turns
    private static final double TURRET_MAX_ACCELERATION = 1.5; // Max power change per loop

    // Low-pass filter for smoothing
    private static final double FILTER_ALPHA = 0.7; // 0=all history, 1=no filtering

    // PID state variables
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastTx = 0.0;
    private double filteredTx = 0.0;
    private double lastTurretPower = 0.0;
    private long lastLoopTime = 0;
    private static final double INTEGRAL_LIMIT = 0.3; // Prevent integral windup

    // Limelight
    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 24;
    private static final int PIPELINE_ID = 2;

    @Override
    public void init() {
        // Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
        robot = new Robot(hardwareMap,telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
//        turretCR = hardwareMap.get(CRServo.class, "turretServo");
//        turretCR.setPower(0.0);
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(PIPELINE_ID);
//        limelight.start();
//
//        lastLoopTime = System.nanoTime();
    }

    public void buildPaths() {
        goToThirdPattern = new Path(new BezierLine(startPose, thirdPattern));
        goToThirdPattern.setLinearHeadingInterpolation(startPose.getHeading(), thirdPattern.getHeading());

        getThirdPattern = new Path(new BezierLine(thirdPattern, thirdPatternPickUp));
        getThirdPattern.setConstantHeadingInterpolation(thirdPatternPickUp.getHeading());

        shootStack1 = new Path(new BezierLine(thirdPattern, shootingPose));
        shootStack1.setConstantHeadingInterpolation(shootingPose.getHeading());

        goToSecondPattern = new Path(new BezierLine(shootingPose, secondPattern));
        goToSecondPattern.setLinearHeadingInterpolation(shootingPose.getHeading(), secondPattern.getHeading());

        getSecondPattern = new Path(new BezierLine(secondPattern, secondPatternPickUp));
        getSecondPattern.setConstantHeadingInterpolation(secondPatternPickUp.getHeading());

        shootStack2 = new Path(new BezierLine(secondPattern, shootingPose));
        shootStack2.setConstantHeadingInterpolation(shootingPose.getHeading());

        goToFirstPattern = new Path(new BezierLine(shootingPose, firstPattern));
        goToFirstPattern.setLinearHeadingInterpolation(shootingPose.getHeading(), thirdPattern.getHeading());

        getFirstPattern = new Path(new BezierLine(firstPattern, firstPatternPickUp));
        getFirstPattern.setConstantHeadingInterpolation(firstPatternPickUp.getHeading());

        shootStack3 = new Path(new BezierLine(firstPattern, shootingPose));
        shootStack3.setConstantHeadingInterpolation(shootingPose.getHeading());

        endingAuton = new Path(new BezierLine(shootingPose, finalPose));
        endingAuton.setConstantHeadingInterpolation(shootingPose.getHeading());
//

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        follower.setMaxPower(1.0);
    }

    @Override
    public void loop() {
//        // Calculate loop time for derivative
//        long currentTime = System.nanoTime();
//        double dt = (currentTime - lastLoopTime) / 1e9; // seconds
//        lastLoopTime = currentTime;
//        dt = Math.max(dt, 0.001); // Prevent division by zero
//
//        LLResult result = limelight.getLatestResult();
//        boolean trackingTag = false;
//        double tx = 0.0;
//        int detectedTagID = -1;
//
//        if (result != null && result.isValid()) {
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//
//            if (fiducials != null && !fiducials.isEmpty()) {
//                for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
//                        tx = fiducial.getTargetXDegrees();
//                        detectedTagID = fiducial.getFiducialId();
//                        trackingTag = true;
//                        break;
//                    }
//                }
//
//                if (!trackingTag && !fiducials.isEmpty()) {
//                    detectedTagID = fiducials.get(0).getFiducialId();
//                }
//            }
//        }
//
//        if (trackingTag) {
//            // Low-pass filter to smooth noisy measurements
//            filteredTx = FILTER_ALPHA * tx + (1 - FILTER_ALPHA) * filteredTx;
//
//            // PID calculation
//            double error = filteredTx;
//
//            // Proportional term
//            double pTerm = TURRET_KP * error;
//
//            // Integral term with anti-windup
//            integralSum += error * dt;
//            integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
//            double iTerm = TURRET_KI * integralSum;
//
//            // Derivative term (rate of change of error)
//            double derivative = (error - lastError) / dt;
//            double dTerm = TURRET_KD * derivative;
//
//            // Combine PID terms
//            double turretPower = pTerm + iTerm + dTerm;
//
//            // Apply minimum power threshold to overcome friction
//            if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
//                turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
//            }
//
//            // Clamp to maximum power
//            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));
//
//            // Velocity limiting - prevent sudden power changes
//            double powerChange = turretPower - lastTurretPower;
//            if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
//                turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
//            }
//
//            // Apply deadzone for lock-on
//            if (Math.abs(error) < TURRET_DEADZONE) {
//                turretCR.setPower(0);
//                integralSum = 0; // Reset integral when locked
//                telemetry.addData("Turret Status", "🎯 LOCKED ON TARGET");
//            } else {
//                turretCR.setPower(turretPower);
//                telemetry.addData("Turret Status", "🔄 TRACKING");
//            }
//
//            // Update state for next loop
//            lastError = error;
//            lastTurretPower = turretPower;
//            lastTx = tx;
//
//            telemetry.addData("Turret Mode", "AUTO (Tag 24)");
//            telemetry.addData("Raw Error", "%.2f°", tx);
//            telemetry.addData("Filtered Error", "%.2f°", filteredTx);
//            telemetry.addData("P | I | D", "%.3f | %.3f | %.3f", pTerm, iTerm, dTerm);
//            telemetry.addData("Turret Power", "%.3f", turretPower);
//
//        } else {
//            // Reset PID when target lost
//            integralSum = 0;
//            lastError = 0;
//            filteredTx = 0;
//        }

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
                robot.shooter.startAutonFarShoot();
                setPathState(1);
                break;
            case 1:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 3000) {
                    robot.intake.shootArtifacts();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 4000){ //decrease if necessary
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopTransfer();
                    follower.followPath(goToThirdPattern, true);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 1000) {
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
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 1000) {
                    robot.shooter.startAutonFarShoot();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 3000) {
                    robot.intake.shootArtifacts();
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 3000){
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopTransfer();
                    follower.followPath(goToSecondPattern, true);
                    setPathState(61);
                }
                break;
            case 61:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
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
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 3000) {
                    robot.shooter.startAutonFarShoot();
                    setPathState(9);
                }
            case 9:
                if (robot.shooter.reachedSpeed()|| pathTimer.getElapsedTime() > 4000) {
                    robot.intake.shootArtifacts();
                   setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 2500){
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopTransfer();
                    follower.followPath(endingAuton, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTime() > 2000)  {
                    setPathState(-1);
                }
                break;
            case 12:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 4000) {
                    robot.shooter.startAutonFarShoot();
                    setPathState(13);
                }
                break;
            case 13:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 4000) {
                    robot.intake.shootArtifacts();
                    setPathState(-1);
                }
                break;





        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

