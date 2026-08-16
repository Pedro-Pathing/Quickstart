package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(group = "4")
public class HoldTest extends OpMode {
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
    }

    @Override
    public void start() {
        follower.hold(new Pose(72, 72, 0));
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Holding?", follower.holding());
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }
}
