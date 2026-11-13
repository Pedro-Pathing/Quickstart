package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.robot;

@TeleOp
public class Drive extends OpMode {
    robot robot = new robot();
    double drivePower = 0.25;
    int shootOn = -1;
    int shootSpeed;
    int turretPosition = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.getTurret().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.aWasReleased()) {
            drivePower = 0.25;
        } else if(gamepad1.bWasReleased()) {
            drivePower = 0.5;
        } else if(gamepad1.yWasReleased()) {
            drivePower = 1;
        }
        if(gamepad1.rightBumperWasReleased()) {
            shootOn *= -1;
        }
        if(shootOn == 1) {
            shootSpeed = 1;
        } else {
            shootSpeed = 0;
        }
        if(gamepad1.dpadRightWasReleased() && turretPosition < 150) {
            turretPosition += 50;
        } else if(gamepad1.dpadLeftWasReleased() && turretPosition > -150) {
            turretPosition -= 50;
        }
        double y = gamepad1.left_stick_y * drivePower;
        double x = -gamepad1.left_stick_x * drivePower;
        double rx = gamepad1.right_stick_x * drivePower;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.setPower(robot.getFL(), frontLeftPower);
        robot.setPower(robot.getBL(), backLeftPower);
        robot.setPower(robot.getFR(), frontRightPower);
        robot.setPower(robot.getBR(), backRightPower);
        robot.setPower(robot.getLeftShoot(), shootSpeed);
        robot.setPower(robot.getRightShoot(), shootSpeed);
        robot.runToPosition(robot.getTurret(), turretPosition, 1, 0);

    }
}
