package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BilordaHoustonField")
public class BilordaHoustonField extends OpMode {

    // Motors
    private DcMotorEx left1, right1, left2, right2;

    // IMU
    private IMU imu;

    @Override
    public void init() {


        left1  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        right1 = hardwareMap.get(DcMotorEx.class, "frontRight");
        left2   = hardwareMap.get(DcMotorEx.class, "backLeft");
        right2  = hardwareMap.get(DcMotorEx.class, "backRight");


        left1.setDirection(DcMotorEx.Direction.REVERSE);
        left2.setDirection(DcMotorEx.Direction.REVERSE);

        right1.setDirection(DcMotorEx.Direction.FORWARD);
        right2.setDirection(DcMotorEx.Direction.FORWARD);

        // Zero power behavior
        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // IMU init
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
    }

    @Override
    public void loop() {

        // Stick input
        double y = -gamepad1.left_stick_y; // forward
        double x =  gamepad1.left_stick_x; // strafe
        double rx = gamepad1.right_stick_x; // rotation

        // Robot heading
        double heading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        // FIELD CENTRIC transform
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // (необязательно) компенсация механического стрейфа
        rotX *= 1.1;

        // Normalize
        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx),
                1
        );

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        // Apply power
        left1.setPower(frontLeftPower);
        left2.setPower(backLeftPower);
        right1.setPower(frontRightPower);
        right2.setPower(backRightPower);

        // Telemetry
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.update();
    }
}