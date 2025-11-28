package org.firstinspires.ftc.teamcode.friends.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Motor Test")
public class  MotorTest extends LinearOpMode {
    private static float power = 0.0f;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("Motor");

        waitForStart();

        if (isStopRequested()) return;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(gamepad1.touchpad) {
                motor.setPower(power);
            } else {
                motor.setPower(0);
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                power += 0.1f;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                power -= 0.1f;
            }

            power = Math.min(1.0f, power);
            power = Math.max(-1.0f, power);

            telemetry.addData("Power: ", power);
            telemetry.update();
        }
    }
}
