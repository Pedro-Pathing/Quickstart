package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ShooterlessAuto", group = "Autonomous")
public class ShooterlessAutoBlueBig extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    ElapsedTime shoot, retrieve;
    private int pathState;

    private final Pose startPose = new Pose(122, 124, Math.toRadians(36));
    private final Pose shootPose = new Pose(121, 124, Math.toRadians(36));
    private final Pose setupPose1 = new Pose(96, 84, Math.toRadians(180));
    private final Pose setupPose2 = new Pose(96, 60, Math.toRadians(180));
    private final Pose setupPose3 = new Pose(96, 36, Math.toRadians(180));
    private final Pose pickupPose1 = new Pose(121, 84, Math.toRadians(180));
    private final Pose pickupPose2 = new Pose(121, 60, Math.toRadians(180));
    private final Pose pickupPose3 = new Pose(121, 36, Math.toRadians(180));



    DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");




    public void Retrieve() {
        retrieve.startTime();
        if (retrieve.seconds() < 4) {
            intakeMotor.setPower(1);
        } else if (retrieve.seconds() > 4) {
            intakeMotor.setPower(0);
            retrieve.reset();
        }
    }

    public void ShootingPath (Pose pickupPose) {
        PathChain ShootPath;
        ShootPath = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();
        follower.followPath(ShootPath);
    }

    public void SetupAndRetrieve(Pose setupPose, Pose pickupPose, Pose Shoot_Start) {
        PathChain SetupPath;
        PathChain RetrievePath;
        SetupPath = follower.pathBuilder()
                .addPath(new BezierLine(Shoot_Start, setupPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), setupPose1.getHeading())
                .build();
        RetrievePath = follower.pathBuilder()
                .addPath(new BezierLine(setupPose,pickupPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        follower.followPath(SetupPath);
        Retrieve();
        follower.followPath(RetrievePath);
    }






    @Override
    public void loop() {

    }
    @Override
    public void init() {
        SetupAndRetrieve(setupPose1,pickupPose1, startPose);
        ShootingPath(pickupPose1);
        SetupAndRetrieve(setupPose2, pickupPose2, shootPose);
        ShootingPath(pickupPose2);
        SetupAndRetrieve(setupPose3, pickupPose3, shootPose);
        ShootingPath(pickupPose3);


    }
}
