package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumTeleOp_Single")
public class code1 extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors from hardware map
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_Motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Motor directions (adjust if needed)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake when joystick released
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to run.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Drive controls
            double y = -gamepad1.left_stick_y;   // Forward/back
            double x = gamepad1.left_stick_x * 1.1; // Strafe boost
            double turn = gamepad1.right_stick_x;   // Rotate

            // Mecanum algorithm
            double fl = y + x + turn;
            double fr = y - x - turn;
            double bl = y - x + turn;
            double br = y + x - turn;

            // Normalize to keep values within [-1, 1]
            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br)));

            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            // Send power to motors
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            telemetry.addData("FL", fl);
            telemetry.addData("FR", fr);
            telemetry.addData("BL", bl);
            telemetry.addData("BR", br);
            telemetry.update();
        }
    }
}
