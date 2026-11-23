package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public class Drivetrain {

    public final DcMotorEx frontLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backLeft;
    public final DcMotorEx backRight;
    public final Follower follower;
    private final Telemetry telemetry;
    private static Pose poseTransfer = new Pose();
    public Drivetrain(Robot robot) {
        follower = Constants.create(robot.hardwareMap);
        frontLeft = robot.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = robot.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = robot.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = robot.hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry = robot.telemetry;
    }

    //robot centric
    public void startDrive(Gamepad gamepad) {
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        DriveCommands.mecanumDrive(
                frontLeft, frontRight, backLeft, backRight, gamepad).schedule();
    }

    //field centric
    public void startDrive(Gamepad gamepad, Follower follower) {
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        DriveCommands.mecanumDriveFieldCentric(
                frontLeft, frontRight, backLeft, backRight, gamepad,
                () -> follower.pose().heading()).schedule();
    }

    public Command periodic() {
        return infinite(() -> {
            follower.update();
            poseTransfer = follower.getPose();

            telemetry.addData("Current X", follower.getPose().getX());
            telemetry.addData("Current Y", follower.getPose().getY());
            telemetry.addData("Current Heading", Math.toDegrees(follower.getHeading()));
            telemetry.addData("Heading Locked", lockHeading);
            telemetry.addData("Heading Target", headingTargetRadians);
        });
    }
}