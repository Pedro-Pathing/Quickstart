package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.subsystems.servoControl;
import org.firstinspires.ftc.teamcode.util.Hardware;

/** @noinspection RedundantThrows*/
@SuppressWarnings("unused")
@TeleOp
public class teleop extends LinearOpMode {

    final boolean DEBUG = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //init hardware + subsystems
        Hardware robot = new Hardware();
        robot.init(hardwareMap);
        intake intake = new intake(robot);
        outtake outtake = new outtake(robot);
        servoControl servoControl = new servoControl(robot);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Driving
            double y = gamepad1.left_stick_y; // Remember, I have no idea why every stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            boolean a = gamepad1.a;

            if (DEBUG) {
                telemetry.addData("Y drive", y);
                telemetry.addData("X drive", x);
                telemetry.addData("RX drive", rx);
            }

            if (gamepad1.y) {
                imu.resetYaw();
                telemetry.addData("Yaw reset", rx);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (DEBUG) {telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));}

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (DEBUG) {
                telemetry.addData("Front Left Power", frontLeftPower);
                telemetry.addData("Back Left Power", backLeftPower);
                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);
            }







            if (gamepad2.right_bumper) outtake.outtakeCharge();
            if (gamepad2.left_bumper) outtake.stopMotor();
            intake.intakePower(gamepad2.left_stick_y);
            servoControl.servoPower(gamepad2.right_stick_y);
            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backRight.setPower(backRightPower);

            telemetry.update();
        }
    }
}