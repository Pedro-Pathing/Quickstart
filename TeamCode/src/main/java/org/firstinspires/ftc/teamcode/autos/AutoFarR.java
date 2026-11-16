package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous(name = "AutoFarR", group = "Autonomous")
public class AutoFarR extends LinearOpMode {

    public DcMotorEx shoota;
    public DcMotor intake;
    public DcMotor transfer;
    public Servo gate;
    double shootaPowwa;
    double shootaSpeedy = 1550;
    double F = 12.0;
    double P = 0.85;

    //pew pew go code heheheh :]




    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

    private final Pose start = poseFactory.of(133.4017, 10.8219, 180);
    private final Pose path1 = poseFactory.of(74.5442, 48.094, 90);
    private final Pose point2 = poseFactory.of(74.3091, 102.0313, 90.2497);
    private final Pose point3Start = poseFactory.of(74.3091, 102.0313, 90);
    private final Pose point3 = poseFactory.of(84.2194, 116.6268, 270);
    private final Pose point4 = poseFactory.of(101.6068, 117.6382, 270);
    private final Pose point5 = poseFactory.of(118.7536, 47.0527, -90.2553);
    private final Pose point6 = poseFactory.of(133.5071, 47.2521, -179.2256);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                shooterOn(),
                shooterFire(),
                shooterStop(),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            OpModeStorage.autonomousEndPose = follower.pose();
            Scheduler.execute();
            intake = hardwareMap.get(DcMotor.class, "intakeMotor");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            transfer = hardwareMap.get(DcMotor.class, "transferMotor");
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transfer.setDirection(DcMotor.Direction.REVERSE);


            shoota = hardwareMap.get(DcMotorEx.class, "shoota");
            shoota.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoota.setDirection(DcMotor.Direction.FORWARD);
            PIDFCoefficients shootaPID = new PIDFCoefficients(P, 0, 0, F);
            shoota.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shootaPID);



            gate = hardwareMap.get(Servo.class, "gateServo");


            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();


        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }


    private Command shooterOn() {
        return sequential(
                instant(() -> {
                    shootaPowwa = shootaSpeedy;
                    shoota.setVelocity(shootaPowwa);
                }),
                waitMs(2000)
        );
    }
    private Command shooterFire() {
        return sequential(
                instant(() -> {
                    transfer.setPower(0.7);
                    intake.setPower(0.7);
                    gate.setPosition(0.25);
                }),
                waitMs(4000)
        );
    }
    private Command shooterStop() {
        return sequential(
                instant(() -> {
                    shootaPowwa = 0;
                    shoota.setVelocity(shootaPowwa);
                    transfer.setPower(0.0);
                    intake.setPower(0.0);
                    gate.setPosition(0.62);
                }),
                waitMs(100)
        );
    }
    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }
}
