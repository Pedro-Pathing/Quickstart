package org.firstinspires.ftc.teamcode.pedroPathing.constants;

/**
 * TeleOpConstants - Centralized configuration for all TeleOp parameters.
 * This file keeps the OpModes concise and easy to tune.
 */
public class TeleOpConstants {

    // --- Drivetrain Limits ---
    public static final double DRIVE_SPEED_LIMIT = 0.65;
    public static final double TURN_SPEED_LIMIT = 0.375; // Lower limit for rotation
    public static final double DEADZONE = 0.05;

    // --- Heading Hold (Manual Drive) ---
    public static final double HEADING_LOCK_KP = 0.75;
    public static final double HEADING_LOCK_KD = 0.08;
    public static final double HEADING_LOCK_TOLERANCE_RAD = Math.toRadians(1.0); // Changed from 1.5 to match Base.java
    // public static final double HEADING_LOCK_TOLERANCE_RAD = Math.toRadians(1.5);

    // --- AprilTag Alignment (Path-based) ---
    public static final int DESIRED_TAG_ID = 586;
    public static final double DESIRED_DISTANCE = 19.5; // Inches
    public static final double ALIGN_UPDATE_SECONDS = 0.25;
    public static final double ALIGN_MAX_POWER = 0.775;

    // --- Camera Calibration & Offsets ---
    public static final double CAMERA_FORWARD_OFFSET = 0.0;
    public static final double CAMERA_LEFT_OFFSET = -2.0; // 2" Right of center
    public static final double CAMERA_YAW_OFFSET = 0.0;   // Degrees (Set to 10.0 if mechanically tilted CCW)

    // --- Vision Stability Filter ---
    public static final int REQUIRED_STABLE_FRAMES = 4;
    public static final double MAX_TRANSLATION_JUMP = 5.0; // Inches
    public static final double MAX_YAW_JUMP_DEG = 10.0;
    public static final double FILTER_ALPHA = 0.3; // Low-pass filter weight

    // --- Path Update Thresholds ---
    public static final double PATH_UPDATE_XY_INCHES = 1.5; // Changed from 0.75 to match Base.java
    // public static final double PATH_UPDATE_XY_INCHES = 0.75;
    public static final double PATH_UPDATE_HEADING_DEG = 2.0;

    // --- Servo Animation ---
    public static final double SERVO_HOME = 0.0;
    public static final double SERVO_MAX = 1.0;
    public static final double ANIMATION_DURATION_SEC = 6.0; // Changed from 3.0 to match Base.java
    // public static final double ANIMATION_DURATION_SEC = 3.0;
    public static final double ANIMATION_STEP_SEC = 0.25;

    // --- Dashboard Rendering ---
    public static final double DASHBOARD_TAG_X = 0.0;
    public static final double DASHBOARD_TAG_Y = 40.0;
}
