package org.firstinspires.ftc.teamcode.behaviorControl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Forward1Auto extends LinearOpMode {

    // Tunable values
    private static final double LAUNCHER_VELOCITY = 1120;
    private static final long SPIN_UP_MS = 4000;
    private static final long FEED_MS = 3000;
    private static final double DRIVE_POWER = -0.5;
    private static final long DRIVE_MS = 1000;

    @Override
    public void runOpMode() {
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        // Drive motors (names and directions match your Constants file)
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "right_back_drive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(40, 0, 0, 12.5);
        launcher.setVelocity(0);
        windmillServo.setPower(0);

        waitForStart();

        // 1. Spin up the launcher
        launcher.setVelocity(LAUNCHER_VELOCITY);
        sleep(SPIN_UP_MS);

        // 2. Feed the balls
        windmillServo.setPower(1);
        sleep(FEED_MS);

        // 3. Stop the shooter
        launcher.setVelocity(0);
        windmillServo.setPower(0);

        // 4. Drive backward for 1 second
        frontLeft.setPower(DRIVE_POWER);
        frontRight.setPower(DRIVE_POWER);
        backLeft.setPower(DRIVE_POWER);
        backRight.setPower(DRIVE_POWER);
        sleep(DRIVE_MS);

        // 5. Stop everything
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}