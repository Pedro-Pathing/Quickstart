package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PedroPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private slideConstants slide;
    private Servo myServoName;
    private final double HOME_POSITION = 0.0;
    private final double MAX_POSITION  = 1.0;

    public enum PathState {
        DRIVE_STARTPOS_SECOND_POS,
        SECOND_POSE,
        INTAKE_PAUSE,
        LIFT,
        LIFT_DOWN,
        DRIVE_THIRD_POSE,
        DRIVE_CURVE_POSE
    }

    ElapsedTime motorTimer = new ElapsedTime();
    PathState pathState;
    DcMotor intakeMotor;

    // Grid Tile Size = 24 inches
    private final double TILE_SIZE = 24.0;

    // 1. Start facing forward relative to the field (90 degrees)
    private final double START_X = 34.92;
    private final double START_Y = 129.23;
    private final Pose startPose = new Pose(START_X, START_Y, Math.toRadians(90));

    // 2. Right 3 tiles (+72 X), rotate 180 deg CCW. 
    // We use 269.9 instead of 270 to ensure the "shortest path" heading interpolation picks CCW (179.9 deg).
    private final Pose secondPose = new Pose(START_X + (3 * TILE_SIZE), START_Y, Math.toRadians(269.9));

    // 3. Down 1 tile (maintain heading)
    private final Pose thirdPose = new Pose(secondPose.getX(), secondPose.getY() - (1 * TILE_SIZE), Math.toRadians(269.9));

    // 4. Back (Up) 1 tile (+24 Y)
    private final Pose turnPose = new Pose(thirdPose.getX(), thirdPose.getY() + (1 * TILE_SIZE), Math.toRadians(269.9));

    // 5. Bézier Curve Control Point and End Point back home
    // Adjusted heading to 75 degrees to compensate for the 15 degree CCW error, so it lands at 90.
    private final Pose bezierControlPoint = new Pose(START_X - 5.5, turnPose.getY() - 12.0);
    private final Pose bezierEndPoint = new Pose(START_X, START_Y);

    private PathChain driveStartToSecond, driveSecondToThird, driveThirdToTurn, driveTurnToCurve;

    public void buildPaths() {
        // Path 1: Drive Right 3 tiles & rotate 180 deg CCW
        driveStartToSecond = follower.pathBuilder()
                .addPath(new BezierLine(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .build();

        // Path 2: Drive Down 1 tile (maintain heading)
        driveSecondToThird = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, thirdPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .build();

        // Path 3: Drive Back Up 1 tile (maintain heading)
        driveThirdToTurn = follower.pathBuilder()
                .addPath(new BezierLine(thirdPose, turnPose))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), turnPose.getHeading())
                .build();

        // Path 4: Curve back to exact start position & restore original 90 deg heading
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
                    // Slowed down by 25% (0.75 power)
                    follower.followPath(driveSecondToThird, 0.75, true);
                    setPathState(PathState.INTAKE_PAUSE);
                }
                break;

            case INTAKE_PAUSE:
                // Wait for the drive to finish AND the 3-second intake timer
                if (!follower.isBusy() && motorTimer.seconds() > 3) {
                    intakeMotor.setPower(0);
                    setPathState(PathState.LIFT);
                }
                break;

            case LIFT:
                // 1. Command the slide once right when entering this state
                if (pathTimer.getElapsedTimeSeconds() < 0.05) {
                    slide.extendToHigh();
                }

                // 2. Servo Animation: Toggles positions cleanly every 0.25 seconds using time
                if (pathTimer.getElapsedTimeSeconds() < 4) {
                    if (((int)(pathTimer.getElapsedTimeSeconds() * 4)) % 2 == 0) {
                        myServoName.setPosition(MAX_POSITION);
                    } else {
                        myServoName.setPosition(HOME_POSITION);
                    }
                } else {
                    myServoName.setPosition(HOME_POSITION);
                }

                // 3. Wait for slide to be up and servo to finish (3 seconds total)
                if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                    setPathState(PathState.LIFT_DOWN);
                }
                break;

            case LIFT_DOWN:
                // 1. Command the slide down
                if (pathTimer.getElapsedTimeSeconds() < 0.05) {
                    slide.extendToBottom();
                }

                // 2. Wait 2 seconds for it to fall before continuing paths
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
                if (!follower.isBusy()) {
                    telemetry.addLine("Autonomous Finished");
                    requestOpModeStop();
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        // 1. Hardware Map Assignments
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setPower(0);
        myServoName = hardwareMap.get(Servo.class, "myServoName");
        slide = new slideConstants(hardwareMap);

        // 2. Timer Initialization
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // 3. Follower Setup
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        buildPaths();

        // 4. Default State Setup
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

        // Helpful diagnostic telemetry
        telemetry.addData("Current State", pathState);
        telemetry.addData("State Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
