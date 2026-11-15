package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "SampleAutoPathing")
public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_ENDPOS,
        DONE
    }

    PathState pathState;

    // 1. Define your poses here
    private final Pose startPose = new Pose(53.12, 11.40, Math.toRadians(190));
    private final Pose endPose = new Pose(5.1787, 98.7571, Math.toRadians(190));

    // 2. Declare your paths here
    private PathChain driveStartposEndpos;

    public void buildPath() {
        driveStartposEndpos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_ENDPOS:
                follower.followPath(driveStartposEndpos, true);
                setPathState(PathState.DONE);
                break;

            case DONE:
                if (!follower.isBusy()) {
                    // Robot is at endPose and stopped
                }
                break;
        }
    }

    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        // Follower & PID settings
        FollowerConstants followerConstants = new FollowerConstants()
                .mass(13.0)
                .forwardZeroPowerAcceleration(-41.278)
                .lateralZeroPowerAcceleration(-59.7819)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
                .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0, 0.6, 0));

        // Motor names & directions directly in this file
        MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(1)
                .leftFrontMotorName("frontLeftMotor")
                .leftRearMotorName("backLeftMotor")
                .rightFrontMotorName("frontRightMotor")
                .rightRearMotorName("backRightMotor")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .xVelocity(57.87)
                .yVelocity(52.29);

        // Drive encoder localizer using your 4 drive motors
        DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
                .leftFrontMotorName("frontLeftMotor")
                .leftRearMotorName("backLeftMotor")
                .rightFrontMotorName("frontRightMotor")
                .rightRearMotorName("backRightMotor")
                .leftFrontEncoderDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearEncoderDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontEncoderDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearEncoderDirection(DcMotorSimple.Direction.FORWARD)
                .robotWidth(15.0)
                .robotLength(15.0)
                .forwardTicksToInches(0.0221)
                .strafeTicksToInches(0.0221);

        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        // Build follower right here without needing Constants.java
        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

        follower.setStartingPose(startPose);
        buildPath();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_STARTPOS_ENDPOS);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
}
