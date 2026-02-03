package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * BilordaHouston оптимизированный:
 * - Турель управляется только по X через AprilTag
 * - Драйв, шутеры, intake, RGB LED сохраняются
 */
@TeleOp(name="BilordaHouston_AprilTag", group="FTC")
public class testturel extends OpMode {

    /* ================= HARDWARE ================= */
    private DcMotor left1, left2, right1, right2;
    private DcMotor intake;
    private DcMotorEx shut1, shut2;
    private Servo pod;
    private DcMotorEx turret; // турель

    // Цветовые датчики
    private NormalizedColorSensor c1, c2, c3, c4;
    private Servo rgb1, rgb2, rgb3;

    /* ================= DRIVE TUNING ================= */
    private static final double DRIVE_DEADZONE = 0.02;
    private static final double MIN_OUT = 0.10;
    private static final double TURN_GAIN = 1.20;

    /* ================= TURRET VIA APRILTAG ================= */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private static final double TURRET_KP = 0.02;
    private static final double TURRET_MAX_POWER = 0.4;
    private static final double TURRET_DEADZONE_CM = 1.5; // см

    /* ================= TURRET STATE ================= */
    private ElapsedTime loopTimer = new ElapsedTime();

    /* ================= SHOOTER ================= */
    private boolean farShooterActive = false;
    private boolean midShooterActive = false;
    private boolean lastYButtonState = false;
    private boolean lastXButtonState = false;

    private static final double FAR_SHOOTER_P = 53;
    private static final double FAR_SHOOTER_F = 6;
    private static final double FAR_SHOOTER_VELOCITY = 1450;

    private static final double MID_SHOOTER_P = 27;
    private static final double MID_SHOOTER_F = 4;
    private static final double MID_SHOOTER_VELOCITY = 1150;

    /* ================= COLOR DETECTION ================= */
    public enum DetectedColor { GREEN, PURPLE, UNKNOWN }

    private static final float MIN_INTENSITY = 0.1f;
    private static final float GREEN_RATIO_THRESHOLD = 0.35f;
    private static final float BLUE_RATIO_THRESHOLD = 0.35f;

    private static final double SERVO_POSITION_GREEN = 0.25;
    private static final double SERVO_POSITION_PURPLE = 0.66;
    private static final double SERVO_POSITION_RED = 0.1;
    private static final double SERVO_POSITION_OFF = 0.0;

    @Override
    public void init() {

        /* ---------- DRIVE & MECHANISMS ---------- */
        left1 = hardwareMap.get(DcMotor.class, "left1");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        right1 = hardwareMap.get(DcMotor.class, "right2");
        right2 = hardwareMap.get(DcMotor.class, "right1");

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shut1 = hardwareMap.get(DcMotorEx.class, "shut1");
        shut2 = hardwareMap.get(DcMotorEx.class, "shut2");
        pod = hardwareMap.get(Servo.class, "pod");

        shut1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut1.setDirection(DcMotorSimple.Direction.FORWARD);
        shut2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Начальные PID средний шутер
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(MID_SHOOTER_P, 0, 0, MID_SHOOTER_F);
        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        /* ---------- TURRET ---------- */
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(0);

        /* ---------- APRILTAG ---------- */
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawTagID(true)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "WebCam"))
                    .addProcessor(aprilTag)
                    .build();

            telemetry.addLine("✅ AprilTag ready");
        } catch (Exception e) {
            telemetry.addLine("⚠️ AprilTag init error: " + e.getMessage());
        }

        /* ---------- COLOR SENSORS & RGB ---------- */
        try {
            c1 = hardwareMap.get(NormalizedColorSensor.class, "c1");
            c2 = hardwareMap.get(NormalizedColorSensor.class, "c2");
            c3 = hardwareMap.get(NormalizedColorSensor.class, "c3");
            c4 = hardwareMap.get(NormalizedColorSensor.class, "c4");

            if (c1 != null) c1.setGain(8);
            if (c2 != null) c2.setGain(8);
            if (c3 != null) c3.setGain(8);
            if (c4 != null) c4.setGain(8);

            rgb1 = hardwareMap.get(Servo.class, "rgb1");
            rgb2 = hardwareMap.get(Servo.class, "rgb2");
            rgb3 = hardwareMap.get(Servo.class, "rgb3");



        } catch (Exception e) {
            telemetry.addLine("⚠️ Color sensors/RGB init error: " + e.getMessage());
        }

        telemetry.addLine("✅ BilordaHouston_AprilTag готов");
        telemetry.update();
    }

    @Override
    public void loop() {

        /* ================= DRIVE ================= */
        double y = -gamepad1.right_stick_x;
        double x = -gamepad1.left_stick_y;
        double r = gamepad1.left_stick_x;

        y = applyDeadzone(y, DRIVE_DEADZONE);
        x = applyDeadzone(x, DRIVE_DEADZONE);
        r = applyDeadzone(r, DRIVE_DEADZONE);

        y = applyMinOutput(y, MIN_OUT);
        x = applyMinOutput(x, MIN_OUT);
        r = applyMinOutput(r, MIN_OUT);

        r *= TURN_GAIN;

        if (y == 0 && x == 0 && r == 0) {
            left1.setPower(0); left2.setPower(0);
            right1.setPower(0); right2.setPower(0);
        } else {
            double lf = y + x + r;
            double lb = y - x + r;
            double rf = y - x - r;
            double rb = y + x - r;

            double max = Math.max(1.0,
                    Math.max(Math.abs(lf), Math.max(Math.abs(lb),
                            Math.max(Math.abs(rf), Math.abs(rb)))));

            left1.setPower(lf / max);
            left2.setPower(lb / max);
            right1.setPower(rf / max);
            right2.setPower(rb / max);
        }

        /* ================= INTAKE ================= */
        if (gamepad1.left_trigger > 0 || gamepad2.right_bumper) intake.setPower(-0.5);
        else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) intake.setPower(0.5);
        else intake.setPower(0.0);

        /* ================= SHOOTER ================= */
        handleShooter();

        /* ================= POD ================= */
        pod.setPosition(gamepad2.a ? 0.7 : 0.5);

        /* ================= TURRET - APRILTAG X ================= */
        if (visionPortal != null && turret != null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);
                double xError = tag.ftcPose.x; // см, влево/вправо

                double power = 0;
                if (Math.abs(xError) > TURRET_DEADZONE_CM) {
                    power = Range.clip(xError * TURRET_KP, -TURRET_MAX_POWER, TURRET_MAX_POWER);
                }

                turret.setPower(power);

                telemetry.addData("AprilTag ID", tag.id);
                telemetry.addData("X error (cm)", xError);
                telemetry.addData("Turret Power", power);
            } else {
                turret.setPower(0);
                telemetry.addLine("AprilTag не найден");
            }
        }

        /* ================= COLOR SENSOR ================= */
        updateColorsAndLEDs();

        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.setPower(0);
        if (visionPortal != null) visionPortal.close();

    }

    /* ================= HELPER FUNCTIONS ================= */
    private void handleShooter() {
        boolean currentYButton = gamepad2.y;
        if (currentYButton && !lastYButtonState) {
            farShooterActive = !farShooterActive;
            if (farShooterActive) midShooterActive = false;
        }
        lastYButtonState = currentYButton;

        boolean currentXButton = gamepad2.x;
        if (currentXButton && !lastXButtonState) {
            midShooterActive = !midShooterActive;
            if (midShooterActive) farShooterActive = false;
        }
        lastXButtonState = currentXButton;

        if (farShooterActive) {
            shut1.setVelocity(FAR_SHOOTER_VELOCITY);
            shut2.setVelocity(-FAR_SHOOTER_VELOCITY);
        } else if (midShooterActive) {
            shut1.setVelocity(MID_SHOOTER_VELOCITY);
            shut2.setVelocity(-MID_SHOOTER_VELOCITY);
        } else {
            shut1.setVelocity(0);
            shut2.setVelocity(0);
        }
    }

    private void updateColorsAndLEDs() {
        if (c1 == null || c2 == null || c3 == null || c4 == null) return;

        DetectedColor color1 = detectColor(c1);
        DetectedColor color2 = detectColor(c2);
        DetectedColor color3 = detectColor(c3);
        DetectedColor color4 = detectColor(c4);

        updateRGBLED(rgb1, color1);
        updateRGBLED(rgb2, color2);
        updateRGBLED(rgb3, combineColors(color3, color4));
    }

    private DetectedColor detectColor(NormalizedColorSensor sensor) {
        if (sensor == null) return DetectedColor.UNKNOWN;

        NormalizedRGBA colors = sensor.getNormalizedColors();
        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;

        float total = normRed + normGreen + normBlue;
        if (total < MIN_INTENSITY) return DetectedColor.UNKNOWN;

        float greenRatio = normGreen / total;
        float blueRatio = normBlue / total;

        boolean isGreen = (normGreen > normRed * 1.2 && normGreen > normBlue * 1.2 && greenRatio > GREEN_RATIO_THRESHOLD);
        boolean isPurple = (normBlue >= normRed * 0.6 && (normRed + normBlue) > normGreen * 1.2 && blueRatio > BLUE_RATIO_THRESHOLD);

        if (isGreen) return DetectedColor.GREEN;
        else if (isPurple) return DetectedColor.PURPLE;
        else return DetectedColor.UNKNOWN;
    }

    private DetectedColor combineColors(DetectedColor c1, DetectedColor c2) {
        if (c1 == DetectedColor.GREEN || c2 == DetectedColor.GREEN) return DetectedColor.GREEN;
        if (c1 == DetectedColor.PURPLE || c2 == DetectedColor.PURPLE) return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }

    private void updateRGBLED(Servo rgbLED, DetectedColor color) {
        if (rgbLED == null) return;
        double pos = SERVO_POSITION_OFF;
        switch (color) {
            case GREEN: pos = SERVO_POSITION_GREEN; break;
            case PURPLE: pos = SERVO_POSITION_PURPLE; break;
            case UNKNOWN: pos = SERVO_POSITION_RED; break;
        }
        rgbLED.setPosition(pos);
    }

    private double applyDeadzone(double v, double dz) { return Math.abs(v) < dz ? 0 : v; }
    private double applyMinOutput(double v, double minOut) {
        if (v == 0) return 0;
        double sign = Math.signum(v);
        return sign * (minOut + (1.0 - minOut) * Math.abs(v));
    }
}