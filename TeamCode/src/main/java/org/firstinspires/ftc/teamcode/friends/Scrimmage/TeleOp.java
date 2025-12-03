package org.firstinspires.ftc.teamcode.friends.Scrimmage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.friends.HardwareMap;

// TODO: Send video of intake and shooter working
//       Test intake and shooter separately
//       Integrate them into one opMode
//       Alter code to have 2 gamepads

// One OpMode for driving
// One OpMode for intake and shooting

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode  {

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMap hwMap = new HardwareMap(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        double drivingSpeedModifier = 0.8;
        double intakePower = 0;
        double shooterPower = 0;

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){

            if(gamepad2.touchpad && drivingSpeedModifier == 0.8){
                drivingSpeedModifier = 1.0;
            }
            else if(gamepad2.touchpad && drivingSpeedModifier == 1.0){
                drivingSpeedModifier = 0.8;
            }

            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            hwMap.frontLeftMotor.setPower(frontLeftPower * drivingSpeedModifier);
            hwMap.backLeftMotor.setPower(backLeftPower * drivingSpeedModifier);
            hwMap.frontRightMotor.setPower(frontRightPower * drivingSpeedModifier);
            hwMap.backRightMotor.setPower(backRightPower * drivingSpeedModifier);

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            /// Shooter
        /*
            if(gamepad1.right_bumper) {
                hwMap.shooterMotor1.setPower(shooterPower);
                hwMap.shooterMotor2.setPower(shooterPower);
            } else {
                hwMap.shooterMotor1.setPower(0);
                hwMap.shooterMotor2.setPower(0);
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
            */

            /// Intake
            if(gamepad1.left_bumper) {
                hwMap.intakeMotor.setPower(intakePower);
            } else {
                hwMap.intakeMotor.setPower(0);
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