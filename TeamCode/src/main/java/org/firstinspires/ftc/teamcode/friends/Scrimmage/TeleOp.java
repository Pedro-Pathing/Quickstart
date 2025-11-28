package org.firstinspires.ftc.teamcode.friends.Scrimmage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode  {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FLM");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BLM");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRM");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BRM");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor shooterMotor = hardwareMap.dcMotor.get("Shooter");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("Intake");

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        double drivingSpeedModifier = 0.8;
        double intakePower = 0;
        double shooterPower = 0;

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){

            /// Driving
            if(gamepad1.touchpad && drivingSpeedModifier == 0.8){
                drivingSpeedModifier = 1.0;
            }
            else if(gamepad1.touchpad && drivingSpeedModifier == 1.0){
                drivingSpeedModifier = 0.8;
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * drivingSpeedModifier);
            backLeftMotor.setPower(backLeftPower * drivingSpeedModifier);
            frontRightMotor.setPower(frontRightPower * drivingSpeedModifier);
            backRightMotor.setPower(backRightPower * drivingSpeedModifier);

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            /// Shooter
            if(gamepad1.right_bumper) {
                shooterMotor.setPower(shooterPower);
            } else {
                shooterMotor.setPower(0);
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                shooterPower += 0.1f;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                shooterPower -= 0.1f;
            }

            shooterPower = Math.min(1.0f, shooterPower);
            shooterPower = Math.max(-1.0f, shooterPower);

            telemetry.addData("Shooter Power: ", shooterPower);
            telemetry.update();

            /// Intake
            if(gamepad1.left_bumper) {
                intakeMotor.setPower(intakePower);
            } else {
                intakeMotor.setPower(0);
            }

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                intakePower += 0.1f;
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_right){
                intakePower -= 0.1f;
            }

            intakePower = Math.min(1.0f, intakePower);
            intakePower = Math.max(-1.0f, intakePower);

            telemetry.addData("Intake Power: ", intakePower);
            telemetry.update();
        }
    }
}