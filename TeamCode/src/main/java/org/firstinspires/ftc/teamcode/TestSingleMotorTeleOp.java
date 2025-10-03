package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MainTeleOp", group="Linear Opmode")
public class TestSingleMotorTeleOp extends LinearOpMode {

    private DcMotor motor1;

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");

        motor1.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            double power = gamepad1.left_stick_y;
            if (power > -.05 && power < .05) {
                power = 0;
            }
            motor1.setPower(power);

            telemetry.addData("Joystick X", power);
            telemetry.addData("Motor Power", motor1.getPower());
            telemetry.update();
        }
    }
}
