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

@Autonomous(name = "Point A to B", group = "Autonomous")
public class PointAToB extends LinearOpMode {

    // Pedro Visualizer coordinates (inches and degrees).
    private static final double START_X = 53.1025;
    private static final double START_Y = 11.4963;
    private static final double START_HEADING_DEG = 90.0;

    private static final double TARGET_X = 6.7335;
    private static final double TARGET_Y = 105.7081;
    private static final double TARGET_HEADING_DEG = 0.0;

    // These are the drive values from your TeleOp. Confirm the wheel diameter
    // and encoder ticks/rev for your exact drivetrain before running.
    private static final double DRIVE_TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_IN = 3.78; // 96 mm wheel
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