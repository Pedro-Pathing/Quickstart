package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Red path auto (robot starts facing backward, so all driving is reversed):
 *   1. Shoot preloaded pollen for 10 seconds (from the start position).
 *   2. Follow the red path using ONLY driving and turns (no strafing):
 *        back -> turn left 90 -> back -> turn right 90
 *        -> back -> turn left 90 -> back
 *
 * Distances below were measured from the field picture (24 in per tile).
 * Tune them on the real field.
 */
@Autonomous(name = "Red Path Auto", group = "Red")
public class RedPathAuto extends LinearOpMode {

    // ===================== START ORIENTATION =====================
    // Robot starts facing backward: drive every leg in reverse.
    static final boolean START_REVERSED = true;

    // ===================== PATH DISTANCES (inches) =====================
    static final double LEG_1_INCHES = 26;   // straight up from the start
    static final double LEG_2_INCHES = 41;   // across toward the red side
    static final double LEG_3_INCHES = 65;   // up the red side
    static final double LEG_4_INCHES = 11;   // final move toward the red goal

    // ===================== SHOOTING =====================
    static final double SHOOT_TOTAL_SECONDS  = 10.0;  // total shooting time
    static final double SHOOTER_SPINUP_SECONDS = 2.0; // wheel spin-up before feeding
    static final double SHOOTER_RPM          = 3500;  // match your Teleop
    static final double SHOOTER_TICKS_PER_REV = 28.0;

    // ===================== DRIVE CONSTANTS =====================
    static final double DRIVE_TICKS_PER_REV = 537.7;   // same as Teleop - change to match motors
    static final double WHEEL_DIAMETER_MM   = 104.0;   // goBILDA 104mm mecanum
    // Motor turns per wheel turn (belt ratio). 1.0 if belts are 1:1.
    static final double MOTOR_TURNS_PER_WHEEL_TURN = 1.0;
    // If a 24 in run measures 22 in, use 24.0 / 22.0
    static final double DISTANCE_CORRECTION = 1.0;
    static final double DRIVE_RPM = 190;               // same limit as Teleop
    static final double DRIVE_TIMEOUT_SECONDS = 6.0;

    // ===================== TURN CONSTANTS =====================
    static final double TURN_KP = 0.015;
    static final double TURN_MIN_POWER = 0.15;
    static final double TURN_MAX_POWER = 0.45;
    static final double TURN_TOLERANCE_DEGREES = 2.0;
    static final double TURN_TIMEOUT_SECONDS = 4.0;

    DcMotorEx frontLeft, backLeft, frontRight, backRight;
    DcMotor intakeMotor, transferMotor;
    DcMotorEx shooterMotor;
    IMU imu;
    double targetHeading = 0; // degrees, tracked across turns

    @Override
    public void runOpMode() throws InterruptedException {
        // Same names/swap/directions as Teleop so everything matches
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        transferMotor = hardwareMap.dcMotor.get("transferMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        telemetry.addData("Status", "Ready - robot starts facing backward");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetYaw();
        targetHeading = 0;

        try {
            // ---------- 1) SHOOT FOR 10 SECONDS ----------
            shootPreloads();

            // ---------- 2) RED PATH (turns + driving only, driving is reversed) ----------
            driveForward(LEG_1_INCHES);
            turnByDegrees(90);    // left
            driveForward(LEG_2_INCHES);
            turnByDegrees(-90);   // right
            driveForward(LEG_3_INCHES);
            turnByDegrees(90);    // left
            driveForward(LEG_4_INCHES);
        } finally {
            stopDrive();
            shooterMotor.setVelocity(0);
            transferMotor.setPower(0);
            intakeMotor.setPower(0);
        }
    }

    // ===================== SHOOTING =====================
    private void shootPreloads() {
        ElapsedTime timer = new ElapsedTime();
        shooterMotor.setVelocity(rpmToTicksPerSec(SHOOTER_RPM, SHOOTER_TICKS_PER_REV));

        while (opModeIsActive() && timer.seconds() < SHOOT_TOTAL_SECONDS) {
            // After spin-up, run transfer to feed pollen into the shooter
            if (timer.seconds() >= SHOOTER_SPINUP_SECONDS) {
                transferMotor.setPower(1.0);
            }
            telemetry.addData("Shooting", "%.1f / %.1f s", timer.seconds(), SHOOT_TOTAL_SECONDS);
            telemetry.addData("Shooter RPM", "%.0f",
                    shooterMotor.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV);
            telemetry.update();
            idle();
        }

        shooterMotor.setVelocity(0);
        transferMotor.setPower(0);
    }

    // ===================== DRIVE (encoders) =====================
    // Drives the given distance. With START_REVERSED the robot moves backward.
    private void driveForward(double inches) {
        if (!opModeIsActive()) return;

        double wheelCircumferenceIn = Math.PI * WHEEL_DIAMETER_MM / 25.4;
        double ticksPerInch = DRIVE_TICKS_PER_REV * MOTOR_TURNS_PER_WHEEL_TURN / wheelCircumferenceIn;
        int target = (int) Math.round(inches * ticksPerInch * DISTANCE_CORRECTION);
        if (START_REVERSED) target = -target;
        double velocity = rpmToTicksPerSec(DRIVE_RPM, DRIVE_TICKS_PER_REV);

        DcMotorEx[] motors = {frontLeft, backLeft, frontRight, backRight};

        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotorEx m : motors) {
            m.setVelocity(velocity);
        }

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {
            boolean anyBusy = false;
            for (DcMotorEx m : motors) {
                anyBusy |= m.isBusy();
            }
            if (!anyBusy) break;
            telemetry.addData("Driving", "%.1f in", START_REVERSED ? -inches : inches);
            telemetry.update();
            idle();
        }

        stopDrive();
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        sleep(150);
    }

    // ===================== TURN (IMU) =====================
    // Positive degrees = turn LEFT (counterclockwise), negative = turn RIGHT.
    private void turnByDegrees(double degrees) {
        if (!opModeIsActive()) return;

        targetHeading = AngleUnit.normalizeDegrees(targetHeading + degrees);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < TURN_TIMEOUT_SECONDS) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(targetHeading - yaw);

            if (Math.abs(error) < TURN_TOLERANCE_DEGREES) break;

            double power = Math.min(TURN_MAX_POWER,
                    Math.max(TURN_MIN_POWER, Math.abs(error) * TURN_KP));
            power = Math.copySign(power, error);

            // error > 0 -> turn left: left side backward, right side forward
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            telemetry.addData("Turning", "target %.0f  yaw %.1f", targetHeading, yaw);
            telemetry.update();
        }

        stopDrive();
        sleep(150);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private static double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }
}