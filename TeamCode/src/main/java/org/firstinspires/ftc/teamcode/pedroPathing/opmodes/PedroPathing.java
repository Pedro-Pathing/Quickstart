package org.firstinspires.ftc.teamcode.pedroPathing.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.slideConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.ServoAnimationHandler;

/**
 * PedroPathing - Refactored Autonomous OpMode.
 * Uses shared components for better organization.
 */
@Autonomous(name = "PedroPathing Auto")
public class PedroPathing extends OpMode {
    
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private slideConstants slide;
    private ServoAnimationHandler servoAnim;

    public enum PathState {
        DRIVE_STARTPOS_SECOND_POS,
        SECOND_POSE,
        INTAKE_PAUSE,
        LIFT,
        LIFT_DOWN,
        DRIVE_THIRD_POSE,
        DRIVE_CURVE_POSE
    }

    private PathState pathState;
    private ElapsedTime motorTimer = new ElapsedTime();
    private DcMotor intakeMotor;

    // --- Pathing Constants ---
    private static final double TILE_SIZE = 24.0;
    private final Pose startPose = new Pose(34.92, 129.23, Math.toRadians(90));
    private final Pose secondPose = new Pose(startPose.getX() + (3 * TILE_SIZE), startPose.getY(), Math.toRadians(269.9));
    private final Pose thirdPose = new Pose(secondPose.getX(), secondPose.getY() - TILE_SIZE, Math.toRadians(269.9));
    private final Pose turnPose = new Pose(thirdPose.getX(), thirdPose.getY() + TILE_SIZE, Math.toRadians(269.9));
    private final Pose bezierControlPoint = new Pose(startPose.getX() - 5.5, turnPose.getY() - 12.0);
    private final Pose bezierEndPoint = new Pose(startPose.getX(), startPose.getY());

    private PathChain driveStartToSecond, driveSecondToThird, driveThirdToTurn, driveTurnToCurve;

    public void buildPaths() {
        driveStartToSecond = follower.pathBuilder()
                .addPath(new BezierLine(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .build();

        driveSecondToThird = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, thirdPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .build();

        driveThirdToTurn = follower.pathBuilder()
                .addPath(new BezierLine(thirdPose, turnPose))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), turnPose.getHeading())
                .build();

        driveTurnToCurve = follower.pathBuilder()
                .addPath(new BezierCurve(turnPose, bezierControlPoint, bezierEndPoint))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SECOND_POS:
                follower.followPath(driveStartToSecond, true);
                setPathState(PathState.SECOND_POSE);
                break;

            case SECOND_POSE:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0.8);
                    motorTimer.reset();
                    follower.followPath(driveSecondToThird, 0.75, true);
                    setPathState(PathState.INTAKE_PAUSE);
                }
                break;

            case INTAKE_PAUSE:
                if (!follower.isBusy() && motorTimer.seconds() > 3) {
                    intakeMotor.setPower(0);
                    setPathState(PathState.LIFT);
                }
                break;

            case LIFT:
                if (pathTimer.getElapsedTimeSeconds() < 0.05) {
                    slide.extendToHigh();
                    servoAnim.startAnimation();
                }
                
                servoAnim.update(false, false);

                if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                    setPathState(PathState.LIFT_DOWN);
                }
                break;

            case LIFT_DOWN:
                if (pathTimer.getElapsedTimeSeconds() < 0.05) slide.extendToBottom();
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    follower.followPath(driveThirdToTurn, 0.4, true);
                    setPathState(PathState.DRIVE_THIRD_POSE);
                }
                break;

            case DRIVE_THIRD_POSE:
                if (!follower.isBusy()) {
                    follower.followPath(driveTurnToCurve, true);
                    setPathState(PathState.DRIVE_CURVE_POSE);
                }
                break;

            case DRIVE_CURVE_POSE:
                if (!follower.isBusy()) requestOpModeStop();
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        slide = new slideConstants(hardwareMap);
        servoAnim = new ServoAnimationHandler(hardwareMap.get(Servo.class, "myServoName"));

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        buildPaths();

        pathState = PathState.DRIVE_STARTPOS_SECOND_POS;
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        slide.update();
        telemetry.addData("State", pathState);
        telemetry.update();
    }
}
