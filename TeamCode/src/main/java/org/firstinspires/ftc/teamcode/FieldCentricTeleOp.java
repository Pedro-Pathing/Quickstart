package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Field Centric TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends LinearOpMode {

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

    // Odometry heading tracking variables (calculated from deadwheels)
    private double botHeading = 0.0;
    private int prevLeftEnc = 0;
    private int prevRightEnc = 0;

    // TODO: Adjust this constant for your robot's track width (encoder counts per radian of rotation)
    private static final double COUNTS_PER_RADIAN = 2000.0; 

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        launcher     = hardwareMap.get(DcMotor.class, "launcher");
        middleIntake = hardwareMap.get(DcMotor.class, "middleIntake");

        leftIntake  = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        conveyor    = hardwareMap.get(CRServo.class, "conveyor");

        // Drivetrain motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoder reading baselines
        prevRightEnc = frontLeft.getCurrentPosition();
        prevLeftEnc = middleIntake.getCurrentPosition();

        telemetry.addData("Status", "Initialized. Press Options on Gamepad 1 to reset Heading.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // --- Reset Heading when Options is pressed on Gamepad 1 ---
            if (gamepad1.options) {
                botHeading = 0.0;
            }

            // --- Subsystem Controls ---
            if (gamepad1.right_trigger > 0.1) {
                launcher.setPower(0.5);
            } else {
                launcher.setPower(0);
            }

            if (gamepad1.right_bumper) {
                conveyor.setPower(1);
            } else if (gamepad1.left_bumper) {
                conveyor.setPower(-1);
            } else {
                conveyor.setPower(0);
            }

            if (gamepad1.left_trigger > 0.1) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
                middleIntake.setPower(1);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                middleIntake.setPower(0);
            }

            // --- Calculate Heading from Deadwheels (Odometry Math) ---
            int currentRightEnc = frontLeft.getCurrentPosition();
            int currentLeftEnc = middleIntake.getCurrentPosition();
            int currentBottomEnc = frontRight.getCurrentPosition();

            int deltaRight = currentRightEnc - prevRightEnc;
            int deltaLeft = currentLeftEnc - prevLeftEnc;

            prevRightEnc = currentRightEnc;
            prevLeftEnc = currentLeftEnc;

            // Change in heading (delta theta) = (deltaRight - deltaLeft) / trackWidth
            double deltaHeading = (double) (deltaRight - deltaLeft) / COUNTS_PER_RADIAN;
            botHeading += deltaHeading;

            // --- Field-Centric Mecanum Drive Math (using Deadwheel Heading) ---
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // Rotate movement vector by negative bot heading for field-centric orientation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

            // Denominator normalization
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetry output
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(botHeading));
            telemetry.addData("Encoders (L, R, Bottom)", "%d, %d, %d", currentLeftEnc, currentRightEnc, currentBottomEnc);
            telemetry.update();
        }
    }
}
