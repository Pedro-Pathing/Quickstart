package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

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
    private Path driveStartposEndpos;

    public void buildPath() {
        driveStartposEndpos = line(startPose, endPose)
                .linear(startPose.heading(), endPose.heading());
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_ENDPOS:
                if (follower != null) {
                    follower.follow(driveStartposEndpos);
                }
                setPathState(PathState.DONE);
                break;

            case DONE:
                if (follower == null || !follower.isBusy()) {
                    requestOpModeStop();
                    // Robot is at endPose and stopped
                }
                break;
        }
    }

    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(startPose);
        }
        buildPath();
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(PathState.DRIVE_STARTPOS_ENDPOS);
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
        }
        statePathUpdate();
    }
}
