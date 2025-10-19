package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Configuration;

@TeleOp(name = "Motor Directions Tester", group = "Test")
public class Motor_Directions extends OpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private Configuration config = new Configuration();

    @Override
    public void init() {
        // Initialize all drive motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, config.leftFront);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, config.rightFront);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, config.leftRear);
        backRightMotor = hardwareMap.get(DcMotorEx.class, config.rightRear);

        // Set motor directions (adjust these if needed)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Motor Directions Tester Initialized");
        telemetry.addLine("Press buttons to test motors:");
        telemetry.addLine("Y - Front Left");
        telemetry.addLine("X - Front Right");
        telemetry.addLine("A - Back Left");
        telemetry.addLine("B - Back Right");
        telemetry.addLine("D-Pad Up - All Motors Forward");
        telemetry.addLine("D-Pad Down - Stop All Motors");
    }

    @Override
    public void loop() {
        // Test front left motor (Y button)
        if (gamepad1.y) {
            frontLeftMotor.setPower(0.5);
            telemetry.addData("Testing", "Front Left Motor");
        } else {
            frontLeftMotor.setPower(0);
        }

        // Test front right motor (X button)
        if (gamepad1.x) {
            frontRightMotor.setPower(0.5);
            telemetry.addData("Testing", "Front Right Motor");
        } else {
            frontRightMotor.setPower(0);
        }

        // Test back left motor (A button)
        if (gamepad1.a) {
            backLeftMotor.setPower(0.5);
            telemetry.addData("Testing", "Back Left Motor");
        } else {
            backLeftMotor.setPower(0);
        }

        // Test back right motor (B button)
        if (gamepad1.b) {
            backRightMotor.setPower(0.5);
            telemetry.addData("Testing", "Back Right Motor");
        } else {
            backRightMotor.setPower(0);
        }

        // Test all motors forward (D-pad up)
        if (gamepad1.dpad_up) {
            frontLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            telemetry.addData("Testing", "All Motors Forward");
        }

        // Stop all motors (D-pad down)
        if (gamepad1.dpad_down) {
            stopAllMotors();
            telemetry.addData("Status", "All Motors Stopped");
        }

        // Display motor powers
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
        
        telemetry.update();
    }

    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    @Override
    public void stop() {
        stopAllMotors();
    }
}
