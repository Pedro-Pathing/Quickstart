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

@Autonomous(name = "Blue Wall Close Auto", group = "Examples")
public class BlueWallCloseAuto extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path scorePreload, intakeStack1, turn, scoreStack1, openGate, initialIntakeStack2, intakeStack2, reverseInitialIntakeStack2, scoreStack2, intakeStack3, scoreStack3;
    private final Pose startPose = new Pose(55, 135, Math.toRadians(270));
    private final Pose intakePose1Control1 = new Pose(44, 72);
    private final Pose intakePose1Contol2 = new Pose(74,85);
    private final Pose scorePose = new Pose(47, 96, Math.toRadians(323));

    private final Pose intakePose1 = new Pose(12, 84, Math.toRadians(180));

    private final Pose openGatePose = new Pose(133, 70, Math.toRadians(180));
    private final Pose openGateControlPoint = new Pose(90,76.5);
    private final Pose initialIntakePose2 = new Pose(84, 60, Math.toRadians(180));
    private final Pose intakePose2Control1 = new Pose(38, 51);
    private final Pose intakePose2Control2 = new Pose(61, 60);
    private final Pose intakePose2 = new Pose(9, 60, Math.toRadians(180));
    private final Pose intakePose3Control1 = new Pose(41, 16);
    private final Pose intakePose3Control2 = new Pose(63, 39);
    private final Pose intakePose3 = new Pose(9, 36, Math.toRadians(180));
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
        turretCR = hardwareMap.get(CRServo.class, "turretServo");
        turretCR.setPower(0.0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_ID);
        limelight.start();
        lastLoopTime = System.nanoTime();
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        openGate = new Path(new BezierCurve(intakePose1, openGateControlPoint, openGatePose));
        intakeStack1 = new Path(new BezierCurve(startPose, intakePose1Control1, intakePose1Contol2, intakePose1));
        scoreStack1 = new Path(new BezierLine(intakePose1, scorePose));
        scoreStack1.setLinearHeadingInterpolation(intakePose1.getHeading(), scorePose.getHeading());
        intakeStack2 = new Path(new BezierCurve(scorePose, intakePose2Control1, intakePose2Control2, intakePose2));
        intakeStack2.setLinearHeadingInterpolation(scorePose.getHeading(), intakePose2.getHeading());
        scoreStack2 = new Path(new BezierLine(intakePose2, scorePose));
        scoreStack2.setLinearHeadingInterpolation(intakePose2.getHeading(), scorePose.getHeading());
        intakeStack3 = new Path(new BezierCurve(scorePose, intakePose3Control1, intakePose3Control2, intakePose3));
        intakeStack3.setLinearHeadingInterpolation(scorePose.getHeading(), intakePose3.getHeading());
        scoreStack3 = new Path(new BezierLine(intakePose3, scorePose));
        scoreStack3.setLinearHeadingInterpolation(intakePose3.getHeading(), scorePose.getHeading());
//

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        follower.setMaxPower(1.0);
        setPathState(0);
    }

    @Override
    public void loop() {
        // Calculate loop time for derivative
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastLoopTime) / 1e9; // seconds
        lastLoopTime = currentTime;
        dt = Math.max(dt, 0.001); // Prevent division by zero
        // -------------------- TURRET CONTROL --------------------
        LLResult result = limelight.getLatestResult();
        boolean trackingTag = false;
        double tx = 0.0;
        int detectedTagID = -1;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                        tx = fiducial.getTargetXDegrees();
                        detectedTagID = fiducial.getFiducialId();
                        trackingTag = true;
                        break;
                    }
                }

                if (!trackingTag && !fiducials.isEmpty()) {
                    detectedTagID = fiducials.get(0).getFiducialId();
                }
            }
        }
        if (trackingTag) {
            // Low-pass filter to smooth noisy measurements
            filteredTx = FILTER_ALPHA * tx + (1 - FILTER_ALPHA) * filteredTx;

            // PID calculation
            double error = filteredTx;

            // Proportional term
            double pTerm = TURRET_KP * error;

            // Integral term with anti-windup
            integralSum += error * dt;
            integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
            double iTerm = TURRET_KI * integralSum;

            // Derivative term (rate of change of error)
            double derivative = (error - lastError) / dt;
            double dTerm = TURRET_KD * derivative;

            // Combine PID terms
            double turretPower = pTerm + iTerm + dTerm;

            // Apply minimum power threshold to overcome friction
            if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
                turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
            }

            // Clamp to maximum power
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            // Velocity limiting - prevent sudden power changes
            double powerChange = turretPower - lastTurretPower;
            if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
                turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
            }

            // Apply deadzone for lock-on
            if (Math.abs(error) < TURRET_DEADZONE) {
                turretCR.setPower(0);
                integralSum = 0; // Reset integral when locked
                telemetry.addData("Turret Status", "🎯 LOCKED ON TARGET");
            } else {
                turretCR.setPower(turretPower);
                telemetry.addData("Turret Status", "🔄 TRACKING");
            }

            // Update state for next loop
            lastError = error;
            lastTurretPower = turretPower;
            lastTx = tx;

            telemetry.addData("Turret Mode", "AUTO (Tag 24)");
            telemetry.addData("Raw Error", "%.2f°", tx);
            telemetry.addData("Filtered Error", "%.2f°", filteredTx);
            telemetry.addData("P | I | D", "%.3f | %.3f | %.3f", pTerm, iTerm, dTerm);
            telemetry.addData("Turret Power", "%.3f", turretPower);

        } else {
            // Reset PID when target lost
            integralSum = 0;
            lastError = 0;
            filteredTx = 0;
        }
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("path timer", pathTimer.getElapsedTime());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        // -------------------- TELEMETRY --------------------
        telemetry.addData("Pose X", "%.2f", follower.getPose().getX());
        telemetry.addData("Pose Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Detected Tag", detectedTagID);
        telemetry.addData("Loop Time", "%.1f ms", dt * 1000);
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.shooter.startAutoCloseShoot(); // start shooter for close shots
                setPathState(1);
                break;
            case 1:
                if (robot.shooter.reachCloseSpeed() || pathTimer.getElapsedTime() > 5000) {
                    robot.intake.startIntakeAndTransfer(); // start intake to shoot
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime()>2000) {
                    robot.shooter.stopShoot();
                    robot.intake.stopTransfer();
                    robot.intake.startIntakeOnly();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime()>2000) {
                    follower.followPath(intakeStack1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000){
                    robot.shooter.startMidShoot();
                    follower.followPath(scoreStack1);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000 && robot.shooter.reachMidSpeed()){
                    robot.intake.startIntakeAndTransfer(); //Shoot to score
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime()>2000) {
                    robot.shooter.stopShoot();
                    robot.intake.stopTransfer();
                    robot.intake.startIntakeOnly();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime()>2000){
                    follower.followPath(intakeStack2);
                    robot.shooter.startMidShoot();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000 && robot.shooter.reachMidSpeed()){
                    follower.followPath(scoreStack2);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000){
                    robot.intake.startIntakeAndTransfer(); //shoot to score
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime()>2000) {
                    robot.shooter.stopShoot();
                    robot.intake.stopTransfer();
                    robot.intake.startIntakeOnly();
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime()>2000){
                    follower.followPath(intakeStack3);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000){
                    robot.shooter.startMidShoot();
                    follower.followPath(scoreStack3);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>4000 && robot.shooter.reachMidSpeed()){
                    robot.intake.startIntakeAndTransfer();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime()>4000){
                    robot.shooter.stopShoot();
                    robot.intake.stopIntake();
                    robot.intake.stopTransfer();
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
