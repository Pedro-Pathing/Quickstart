package org.firstinspires.ftc.teamcode.behaviorControl;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Auto Leave Shoot and Park")
public class AutoLeaveShootAndPark extends OpMode {

    public Follower follower;
    public DcMotorEx launcher;
    public CRServo windmillServo;

    public Paths paths;

    public final PoseFactory factory = PoseFactory.degrees();
    public final Pose start = factory.of(88.372, 131.4403, 90);


    public long timer;
    public int state = 0;

    @Override
    public void init() {

        paths = new Paths();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(40, 0, 0, 12.5);

        launcher.setVelocity(0);
        windmillServo.setPower(0);
    }

    @Override
    public void start() {

        launcher.setVelocity(1120);
        timer = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        follower.update();

        long elapsed = System.currentTimeMillis() - timer;

        if (state == 0 && elapsed >= 4000) {

            windmillServo.setPower(1);
            timer = System.currentTimeMillis();
            state = 1;
        }

        else if (state == 1 && elapsed >= 3000) {

            launcher.setVelocity(0);
            windmillServo.setPower(0);

            Path path = paths.path1();
            follower.follow(path);

            state = 2;
        }
    }

    @Override
    public void stop() {

        launcher.setVelocity(0);
        launcher.setPower(0);
        windmillServo.setPower(0);
    }
}