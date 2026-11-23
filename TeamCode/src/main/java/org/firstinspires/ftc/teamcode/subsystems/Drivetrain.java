package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public class Drivetrain {

    public final DcMotorEx frontLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backLeft;
    public final DcMotorEx backRight;
    public final Follower follower;
    private final Telemetry telemetry;

    private Gamepad gamepad1;

    public Drivetrain(OpMode opMode) {
        follower = Constants.create(opMode.hardwareMap);
        follower.setPose(OpModeStorage.autonomousEndPose);
        
        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry = opMode.telemetry;

        gamepad1 = opMode.gamepad1;
    }


    public Command periodic() {
        return infinite(() -> {
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            follower.manual(powers);

            follower.update();

            telemetry.addData("Current X", follower.pose().x());
            telemetry.addData("Current Y", follower.pose().y());
            telemetry.addData("Current Heading", Math.toDegrees(follower.pose().heading()));
        });
    }
}