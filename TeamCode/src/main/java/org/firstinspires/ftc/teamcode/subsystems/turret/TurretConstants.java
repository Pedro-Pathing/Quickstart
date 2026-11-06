package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    public static String SERVO_NAME = "turret";
    public static String ENCODER_NAME = "turretEncoder";

    public static boolean SERVO_REVERSED = true;

    public static boolean ENCODER_REVERSED = false;

    public static double GEAR_RATIO = 25.0 / 20.0;

    public static double TICKS_PER_REV = 8192;
    public static double ENCODER_GEAR_RATIO = 25.0 / 12.0;

    public static double SANITY_MARGIN_DEG = 30;

    public static double TURRET_FWD = 0.318 ;
    public static double TURRET_LEFT = 0;

    public static double AIM_OFFSET_DEG = 0;

    public static double IDLE_ANGLE_DEG = 0;

    public static double STALL_MIN_POWER = 0.3;
    public static double STALL_VEL_DEG_S = 5;
    public static double STALL_TIME_S = 0.5;
    public static double STALL_COOLDOWN_S = 0.5;

    public static double MIN_ANGLE_DEG = -175;
    public static double MAX_ANGLE_DEG = 175;

    public static double kP = 3.2;
    public static double kI = 0.2;
    public static double kD = 0.16;

    public static double kS = 0;
    public static double DEADBAND_DEG = 0.2;

    public static double MAX_POWER = 2;

    public static double VELOCITY_FILTER_ALPHA = 0.3;

    public static double AIM_TOLERANCE_DEG = 2;
    public static double SETTLED_VEL_DEG_S = 15;

    public static boolean USE_VISION = false;
    public static double VISION_MAX_DISAGREE_DEG = 10;
    public static double BIAS_DECAY_PER_S = 0.5;
    public static double VISION_BIAS_ALPHA = 0.2;
    public static double VISION_LOCK_VALID_MS = 500;
    public static boolean ALLOW_ODOMETRY_ONLY_SHOTS = true;

    public static boolean SHOOT_ON_THE_MOVE = true;
    public static double TRANSFER_DELAY_S = 0.3;
    public static double[] TOF_DIST_IN = {24, 48, 72, 96, 120};
    public static double[] TOF_S = {0.35, 0.45, 0.65, 0.65, 0.75};
    public static double MAX_SHOOT_SPEED_IN_S = 30;
    public static double POSE_VELOCITY_ALPHA = 0.3;
    public static double MAX_ROBOT_SPEED_IN_S = 120;

    public static double MIN_SHOT_DIST = 24;
    public static double MAX_SHOT_DIST = 120;

    public static boolean USE_MANUAL_ANGLE = false;
    public static double MANUAL_ANGLE_DEG = 0;
    public static boolean SHOW_TAGS = false;
}
