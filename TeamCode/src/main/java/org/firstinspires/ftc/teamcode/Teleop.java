package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver; // package depends on where your driver file lives
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MainTeleop", group = "Teleop")
public class Teleop extends LinearOpMode {


    static final double DRIVE_TICKS_PER_REV   = 537.7;
    static final double SHOOTER_TICKS_PER_REV = 28.0;

    static final double DRIVE_MAX_RPM           = 190;
    static final double SHOOTER_MAX_RPM         = 3750;

    static final double TRANSFER_SPEED_MULTIPLIER = 1.0;
    static final double SHOOTER_REVERSE_MAX_RPM = 700;
    static double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("frontRightMotor");

        // Mechanisms
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor transferMotor = hardwareMap.dcMotor.get("transferMotor");
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        // FIXED: left side reversed instead of right side
        // (robot was driving backwards when commanded forwards)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // FIXED: intake was spinning the wrong way
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        // Drivetrain uses encoders so velocity can be capped
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooter uses encoder for velocity control
        PIDFCoefficients p = new PIDFCoefficients(80,0,0,0);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, p);

        // Retrieve GoBilda Pinpoint Driver (configured as "imu" in HardwareMap)
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        odo.setOffsets(-18.4, -179.6, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                 GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU(); // robot must be still to calibrate IMU

        waitForStart();

        if (isStopRequested()) return;

        // Max drivetrain velocity (ticks/sec) corresponding to 250 RPM
        double driveMaxTicksPerSec = rpmToTicksPerSec(DRIVE_MAX_RPM, DRIVE_TICKS_PER_REV);

        while (opModeIsActive()) {
            odo.update();

            // ===================== DRIVETRAIN (GAMEPAD 1) =====================
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                odo.resetPosAndIMU();
            }

            double botHeading = odo.getPosition().getHeading(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;



            // Powers (-1..1) are scaled so 1.0 = 250 RPM max
//            ((DcMotorEx) frontLeftMotor).setVelocity(frontLeftPower * driveMaxTicksPerSec);
//            ((DcMotorEx) backLeftMotor).setVelocity(backLeftPower * driveMaxTicksPerSec);
//            ((DcMotorEx) frontRightMotor).setVelocity(frontRightPower * driveMaxTicksPerSec);
//            ((DcMotorEx) backRightMotor).setVelocity(backRightPower * driveMaxTicksPerSec);
            double decrease = 1.0;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower * decrease);
            backRightMotor.setPower(backRightPower * decrease);

            // ===================== SHOOTER CONTROLLER (GAMEPAD 2) =====================
            // LT = intake, LB = outtake (reverse intake)
            if (gamepad2.left_trigger_pressed) {
                intakeMotor.setPower(1.0);
                transferMotor.setPower(1.0);
                shooterMotor.setVelocity(-rpmToTicksPerSec(SHOOTER_REVERSE_MAX_RPM, SHOOTER_TICKS_PER_REV));
            } else if (gamepad2.left_bumper){
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0);
            }
            // B = transfer, X = reverse transfer
            if (gamepad2.b) {
                transferMotor.setPower(TRANSFER_SPEED_MULTIPLIER);
            } else if (gamepad2.x) {
                transferMotor.setPower(-TRANSFER_SPEED_MULTIPLIER);
            } else {
                transferMotor.setPower(0);
            }

            // RT = shooter (up to 4500 RPM), RB = reverse shooter (700 RPM)
            if (gamepad2.right_trigger_pressed) {
                shooterMotor.setVelocity(rpmToTicksPerSec(SHOOTER_MAX_RPM, SHOOTER_TICKS_PER_REV));
            } else if (gamepad2.right_bumper) {
                shooterMotor.setVelocity(-rpmToTicksPerSec(SHOOTER_REVERSE_MAX_RPM, SHOOTER_TICKS_PER_REV));
            } else {
                shooterMotor.setVelocity(0);
            }

            telemetry.addData("Shooter RPM", shooterMotor.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV);
            telemetry.update();
        }
    }
}