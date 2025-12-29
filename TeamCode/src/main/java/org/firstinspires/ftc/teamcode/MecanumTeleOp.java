package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize motors
        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right_motor");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor outtakeMotor = hardwareMap.dcMotor.get("outtake_motor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set motor actions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //------------ Mecanum Drive ------------
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad2.right_bumper) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad2.b) {
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeMotor.setPower(1);
                outtakeMotor.setPower(0);
            } else {
                intakeMotor.setPower(0);
                outtakeMotor.setPower(0);
            }

            if (gamepad2.left_bumper) {
                outtakeMotor.setPower(1);
                intakeMotor.setPower(1);
                outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }else{
                outtakeMotor.setPower(0);
                intakeMotor.setPower(0);
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            if (gamepad2.a) {
                outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                outtakeMotor.setPower(1);
            } else {
                outtakeMotor.setPower(0);
            }
        }
    }
}
