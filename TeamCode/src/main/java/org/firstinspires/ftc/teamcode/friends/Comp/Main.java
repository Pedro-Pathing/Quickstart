package org.firstinspires.ftc.teamcode.friends.Comp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "Driving")
public class Main extends LinearOpMode {
    private static double speedModifier = 0.8;
    private static float power = -0.8f;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FLM");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BLM");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRM");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BRM");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor motor = hardwareMap.dcMotor.get("Motor");

        waitForStart();

        if (isStopRequested()) return;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()) {

            /// Driving

            if(gamepad1.touchpad && speedModifier == 0.8){
                speedModifier = 1.0;
            }
            else if(gamepad1.touchpad && speedModifier == 1.0){
                speedModifier = 0.8;
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * speedModifier);
            backLeftMotor.setPower(backLeftPower * speedModifier);
            frontRightMotor.setPower(frontRightPower * speedModifier);
            backRightMotor.setPower(backRightPower * speedModifier);

            /// Intake

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            boolean intakeTriggered = false;
            if(gamepad1.x) { // toggle so u don't have to hold
                motor.setPower(power);
            } else {
                motor.setPower(0);
            }

            if (currentGamepad1.right_trigger == 1 && !(previousGamepad1.right_trigger == 1)) {
                power += 1f;
            }
            if (currentGamepad1.left_trigger == 1 && !(previousGamepad1.left_trigger == 1)){
                power -= 1f;
            }

            power = Math.min(1.0f, power);
            power = Math.max(-1.0f, power);

            telemetry.addData("Power: ", power);
            telemetry.update();
        }
    }
}