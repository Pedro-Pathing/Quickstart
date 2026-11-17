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

@Autonomous(name = "Auto", group = "Autonomous")
public class auto extends LinearOpMode {

    // Pedro Visualizer coordinates (inches and degrees).
    private static final double START_X = 53.1025;
    private static final double START_Y = 11.4963;
    private static final double START_HEADING_DEG = 90.0;

    private static final double TARGET_X = 6.7335;
    private static final double TARGET_Y = 105.7081;
    private static final double TARGET_HEADING_DEG = 0.0;

    // Drive values matching your drivetrain.
    private static final double DRIVE_TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_IN = 4.09449; // 104 mm wheel
    private static final double DRIVE_POWER = 0.40;

    private static final double TURN_KP = 0.012;
    private static final double TURN_MIN_POWER = 0.12;
    private static final double TURN_MAX_POWER = 0.35;
    private static final double TURN_TOLERANCE_DEG = 2.0;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Drive motor names and directions copied from your TeleOp.
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrakeBehavior();
        resetDriveEncoders();

        // Same REV Hub mounting orientation as your TeleOp.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        telemetry.addLine("Point A to B - encoder + IMU auto");
        telemetry.addLine("No Pedro Pathing is used.");
        telemetry.addLine("Place robot at the visualizer START pose (53.10, 11.50, 90 deg).");
        telemetry.addData("Target", "(%.2f, %.2f, %.1f deg)",
                TARGET_X, TARGET_Y, TARGET_HEADING_DEG);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            stopDrive();
            return;
        }

        // Make the robot's starting IMU heading the zero reference.
        imu.resetYaw();

        // Drive from the START coordinates to the TARGET coordinates.
        boolean reachedPoint = driveFromStartToTarget();
        if (!reachedPoint || !opModeIsActive()) {
            stopDrive();
            return;
        }

        // Turn from the visualizer's starting heading to its target heading.
        double turnDegrees = normalizeDegrees(TARGET_HEADING_DEG - START_HEADING_DEG);
        boolean reachedHeading = turnByDegrees(turnDegrees);

        stopDrive();
        telemetry.addData("Point B reached", reachedPoint);
        telemetry.addData("Target heading reached", reachedHeading);
        telemetry.addData("Final IMU yaw", "%.1f deg", getYawDegrees());
        telemetry.addLine("Auto finished.");
        telemetry.update();
    }

    private void setBrakeBehavior() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void resetDriveEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private double getYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double normalizeDegrees(double degrees) {
        double normalized = degrees;
        while (normalized > 180.0) normalized -= 360.0;
        while (normalized < -180.0) normalized += 360.0;
        return normalized;
    }

    private boolean driveFromStartToTarget() {
        double deltaX = TARGET_X - START_X;
        double deltaY = TARGET_Y - START_Y;

        double distanceInches = Math.hypot(deltaX, deltaY);
        double fieldAngleRad = Math.atan2(deltaY, deltaX);

        // Robot initial heading in field frame is 90 degrees (Math.PI / 2 radians).
        double robotRelativeAngleRad = fieldAngleRad - Math.toRadians(START_HEADING_DEG);

        double rotX = Math.cos(robotRelativeAngleRad) * 1.1; // Strafe compensation factor
        double rotY = Math.sin(robotRelativeAngleRad);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX), 1.0);
        double flP = (rotY + rotX) / denominator;
        double blP = (rotY - rotX) / denominator;
        double frP = (rotY - rotX) / denominator;
        double brP = (rotY + rotX) / denominator;

        double ticksPerInch = DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
        int targetTicksFL = (int) Math.round(flP * distanceInches * ticksPerInch);
        int targetTicksBL = (int) Math.round(blP * distanceInches * ticksPerInch);
        int targetTicksFR = (int) Math.round(frP * distanceInches * ticksPerInch);
        int targetTicksBR = (int) Math.round(brP * distanceInches * ticksPerInch);

        resetDriveEncoders();

        frontLeftMotor.setTargetPosition(targetTicksFL);
        backLeftMotor.setTargetPosition(targetTicksBL);
        frontRightMotor.setTargetPosition(targetTicksFR);
        backRightMotor.setTargetPosition(targetTicksBR);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(flP * DRIVE_POWER);
        backLeftMotor.setPower(blP * DRIVE_POWER);
        frontRightMotor.setPower(frP * DRIVE_POWER);
        backRightMotor.setPower(brP * DRIVE_POWER);

        while (opModeIsActive() && (frontLeftMotor.isBusy() || backLeftMotor.isBusy()
                || frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            idle();
        }

        stopDrive();
        resetDriveEncoders();
        return true;
    }

    private boolean turnByDegrees(double targetDegrees) {
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();

        while (opModeIsActive() && timeout.seconds() < 4.0) {
            double currentYaw = getYawDegrees();
            double error = normalizeDegrees(targetDegrees - currentYaw);

            if (Math.abs(error) <= TURN_TOLERANCE_DEG) {
                stopDrive();
                return true;
            }

            double pPower = error * TURN_KP;
            double sign = Math.signum(pPower);
            double mag = Math.abs(pPower);
            double turnPower = Math.max(TURN_MIN_POWER, Math.min(TURN_MAX_POWER, mag)) * sign;

            frontLeftMotor.setPower(-turnPower);
            backLeftMotor.setPower(-turnPower);
            frontRightMotor.setPower(turnPower);
            backRightMotor.setPower(turnPower);
        }

        stopDrive();
        return false;
    }
}
