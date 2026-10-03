package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto26(1)", group = "Autonomous")
public class Auto26 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor middleIntake = null;
    private DcMotor launcher = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private CRServo conveyor = null;

    // TODO: Tuning Constants (Encoder counts per inch for forward driving and strafing)
    public static double COUNTS_PER_INCH_FORWARD = 500.0;
    public static double COUNTS_PER_INCH_STRAFE = 500.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        ///
//////
        // Initialize hardware
        frontLeft    = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight   = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft     = hardwareMap.get(DcMotor.class, "backLeft");
        backRight    = hardwareMap.get(DcMotor.class, "backRight");
        launcher     = hardwareMap.get(DcMotor.class, "launcher");
        middleIntake = hardwareMap.get(DcMotor.class, "middleIntake");
        leftIntake   = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake  = hardwareMap.get(CRServo.class, "rightIntake");
        conveyor     = hardwareMap.get(CRServo.class, "conveyor");

        // Drivetrain motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized. Ready for Autonomous.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Step 1: Drive forward 54 inches using odometry
            driveForward(54.0, 0.5);

            // Step 2: Strafe left 14 inches using odometry
            strafeLeft(14.0, 0.5);

            telemetry.addData("Status", "Autonomous Complete!");
            telemetry.update();
        }
    }

    /**
     * Drive forward or backward a specified distance in inches using deadwheel odometry.
     * @param inches Distance to drive in inches (positive = forward, negative = reverse)
     * @param power Power level (0.0 to 1.0)
     */
    public void driveForward(double inches, double power) {
        int initialPosition = middleIntake.getCurrentPosition(); // Left deadwheel encoder
        int targetCounts = (int) (inches * COUNTS_PER_INCH_FORWARD);
        int direction = inches >= 0 ? 1 : -1;

        // Apply motor power
        frontLeft.setPower(power * direction);
        backLeft.setPower(power * direction);
        frontRight.setPower(power * direction);
        backRight.setPower(power * direction);

        while (opModeIsActive() && Math.abs(middleIntake.getCurrentPosition() - initialPosition) < Math.abs(targetCounts)) {
            telemetry.addData("Action", "Driving Forward");
            telemetry.addData("Target Inches", inches);
            telemetry.addData("Current Ticks", middleIntake.getCurrentPosition() - initialPosition);
            telemetry.addData("Target Ticks", targetCounts);
            telemetry.update();
        }

        stopMotors();
        sleep(250); // Pause briefly to allow robot to settle
    }

    /**
     * Strafe left or right a specified distance in inches using the perpendicular deadwheel odometry.
     * @param inches Distance to strafe in inches (positive = left, negative = right)
     * @param power Power level (0.0 to 1.0)
     */
    public void strafeLeft(double inches, double power) {
        int initialPosition = frontRight.getCurrentPosition(); // Bottom / Perpendicular deadwheel encoder
        int targetCounts = (int) (inches * COUNTS_PER_INCH_STRAFE);
        int direction = inches >= 0 ? 1 : -1;

        // Mecanum strafe left power directions
        frontLeft.setPower(-power * direction);
        backLeft.setPower(power * direction);
        frontRight.setPower(power * direction);
        backRight.setPower(-power * direction);

        while (opModeIsActive() && Math.abs(frontRight.getCurrentPosition() - initialPosition) < Math.abs(targetCounts)) {
            telemetry.addData("Action", "Strafing Left");
            telemetry.addData("Target Inches", inches);
            telemetry.addData("Current Ticks", frontRight.getCurrentPosition() - initialPosition);
            telemetry.addData("Target Ticks", targetCounts);
            telemetry.update();
        }

        stopMotors();
        sleep(250); // Pause briefly to allow robot to settle
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
