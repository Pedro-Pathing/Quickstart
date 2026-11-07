// AUTO-GENERATED FILE - DO NOT EDIT DIRECTLY
package org.firstinspires.ftc.teamcode.robot.config.generated;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;

@SuppressWarnings("unused")
public final class config {
  private config() {}

  public static Meta meta;
  public static Field field;
  public static Hive hive;
  public static Match match;
  public static Inventory inventory;
  public static Pedro pedro;
  public static Turret turret;
  public static Vision vision;
  public static Sentinel sentinel;
  public static Casablanca casablanca;
  public static Shooter shooter;
  public static Auto auto;
  public static Teleop teleop;
  public static AgentBridge agent_bridge;

  public static String META_SEASON;
  public static double FIELD_SIZE_IN;
  public static double FIELD_HIVE_CENTER_TO_CENTER_IN;
  public static double FIELD_HIVE_PIVOT_HEIGHT_IN;
  public static double FIELD_HIVE_UP_OPENING_OFFSET_IN;
  public static double FIELD_HIVE_UP_OPENING_HEIGHT_IN;
  public static double FIELD_HIVE_DOWN_OPENING_OFFSET_IN;
  public static double FIELD_HIVE_DOWN_OPENING_HEIGHT_IN;
  public static double FIELD_HIVE_FRAME_WIDTH_IN;
  public static double FIELD_HIVE_FRAME_DEPTH_IN;
  public static double FIELD_HIVE_BOTTOM_HEIGHT_IN;
  public static double FIELD_LOADING_ZONE_START_Y_IN;
  public static double FIELD_LOADING_ZONE_WIDTH_IN;
  public static double FIELD_LOADING_ZONE_DEPTH_IN;
  public static double FIELD_GARDEN_LENGTH_IN;
  public static double FIELD_GARDEN_DEPTH_IN;
  public static double FIELD_FLOWER_WALL_OFFSET_IN;
  public static double FIELD_FLOWER_CORNER_OFFSET_IN;
  public static double HIVE_UP_HEIGHT_THRESHOLD_IN;
  public static int HIVE_CONFIRM_FRAMES_COUNT;
  public static int HIVE_SETTLE_MS;
  public static int HIVE_MAX_OBSERVATION_AGE_MS;
  public static int MATCH_AUTO_MS;
  public static int MATCH_TELEOP_MS;
  public static int MATCH_FLOWER_UNLOCK_REMAINING_MS;
  public static int MATCH_FINAL_WARNING_REMAINING_MS;
  public static int INVENTORY_CAPACITY_COUNT;
  public static int INVENTORY_PRELOAD_COUNT;
  public static boolean INVENTORY_ENTRY_SENSOR_ENABLED;
  public static String INVENTORY_ENTRY_SENSOR_NAME;
  public static boolean INVENTORY_ENTRY_SENSOR_ACTIVE_LOW;
  public static int INVENTORY_ENTRY_DEBOUNCE_MS;
  public static double PEDRO_LOCALIZATION_X_POD_OFFSET;
  public static double PEDRO_LOCALIZATION_Y_POD_OFFSET;
  public static double PEDRO_HEADING_PID_P;
  public static double PEDRO_HEADING_PID_I;
  public static double PEDRO_HEADING_PID_D;
  public static double PEDRO_HEADING_KS;
  public static double PEDRO_TRANSLATION_GAIN_THRESHOLD;
  public static double PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_P;
  public static double PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_I;
  public static double PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_D;
  public static double PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_P;
  public static double PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_I;
  public static double PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_D;
  public static double PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_P;
  public static double PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_I;
  public static double PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_D;
  public static double PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_P;
  public static double PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_I;
  public static double PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_D;
  public static double PEDRO_FEEDFORWARD_COAST_KV;
  public static double PEDRO_FEEDFORWARD_BRAKE_KV;
  public static double PEDRO_BRAKING_FORWARD_LINEAR;
  public static double PEDRO_BRAKING_FORWARD_QUADRATIC;
  public static double PEDRO_BRAKING_STRAFE_LINEAR;
  public static double PEDRO_BRAKING_STRAFE_QUADRATIC;
  public static double PEDRO_BRAKING_HEADING_LINEAR;
  public static double PEDRO_BRAKING_HEADING_QUADRATIC;
  public static double PEDRO_MOTION_MAX_FORWARD_VELOCITY;
  public static double PEDRO_MOTION_MAX_STRAFE_VELOCITY;
  public static double PEDRO_MOTION_NATURAL_FORWARD_DECELERATION;
  public static double PEDRO_MOTION_NATURAL_STRAFE_DECELERATION;
  public static double PEDRO_MOTION_MAX_PATH_SPEED;
  public static boolean TURRET_ENABLED;
  public static double TURRET_MAX_POWER_OUTPUT;
  public static boolean TURRET_SERVO_DIRECTION_INVERTED;
  public static double TURRET_KS_POSITIVE;
  public static double TURRET_KS_NEGATIVE;
  public static String TURRET_ORIENTATION_LOGO;
  public static String TURRET_ORIENTATION_USB;
  public static double TURRET_TRAVEL_MIN_ANGLE;
  public static double TURRET_TRAVEL_MAX_ANGLE;
  public static boolean TURRET_STALL_ENABLED;
  public static double TURRET_STALL_POWER_THRESHOLD;
  public static double TURRET_STALL_MIN_PROGRESS_DEG;
  public static double TURRET_STALL_TIMEOUT_SEC;
  public static double TURRET_STALL_RETRY_AFTER_SEC;
  public static boolean TURRET_RUNAWAY_ENABLED;
  public static double TURRET_RUNAWAY_DIVERGENCE_DEG;
  public static double TURRET_PIDF_P;
  public static double TURRET_PIDF_I;
  public static double TURRET_PIDF_D;
  public static double TURRET_PIDF_F;
  public static double TURRET_KV;
  public static double TURRET_FF_MIN_RATE_DPS;
  public static double TURRET_PIVOT_FORWARD_IN;
  public static double TURRET_PIVOT_LEFT_IN;
  public static double TURRET_INTEGRAL_BAND_DEG;
  public static double TURRET_INTEGRAL_LIMIT_POWER;
  public static double TURRET_TOLERANCE_MAX_DRIFT;
  public static double TURRET_TOLERANCE_NEAR_CUTOFF;
  public static double TURRET_TOLERANCE_NEAR_VAL;
  public static double TURRET_TOLERANCE_MIN_DEG;
  public static double TURRET_TOLERANCE_MAX_DEG;
  public static double TURRET_TOLERANCE_EXIT_FACTOR;
  public static boolean TURRET_ANALOG_ENCODER_ENABLED;
  public static double TURRET_ANALOG_ENCODER_ZERO_VOLTAGE;
  public static double TURRET_ANALOG_ENCODER_DEGREES_PER_VOLT;
  public static double TURRET_ANALOG_ENCODER_MIN_VOLTAGE;
  public static double TURRET_ANALOG_ENCODER_MAX_VOLTAGE;
  public static double TURRET_ANALOG_ENCODER_FULL_SCALE_VOLTAGE;
  public static boolean TURRET_ANALOG_ENCODER_INVERTED;
  public static boolean VISION_ENABLED;
  public static boolean VISION_CONTINUOUS;
  public static String VISION_CAMERA_NAME;
  public static double VISION_DECIMATION;
  public static double VISION_CAMERA_FORWARD_IN;
  public static double VISION_CAMERA_LEFT_IN;
  public static double VISION_CAMERA_HEIGHT_IN;
  public static double VISION_CAMERA_YAW_DEG;
  public static double VISION_CAMERA_PITCH_UP_DEG;
  public static double VISION_RELOCALIZE_MAX_RANGE_IN;
  public static double VISION_RELOCALIZE_MAX_CORRECTION_IN;
  public static double SENTINEL_ROBOT_WIDTH;
  public static double SENTINEL_ROTATION_LOOKAHEAD_TIME;
  public static boolean SENTINEL_PROTECT_FULL_HIVE_FOOTPRINT;
  public static double SENTINEL_FRAME_BASE_WIDTH_IN;
  public static double SENTINEL_CENTER_LINE_MARGIN_IN;
  public static double SENTINEL_WALL_CONTACT_TOLERANCE_IN;
  public static double SENTINEL_PARK_LANE_MARGIN_IN;
  public static double SENTINEL_ROUTE_SAMPLE_STEP;
  public static double SENTINEL_PARK_TURN_RATE_DPS;
  public static double SENTINEL_PARK_SETTLE_PER_WAYPOINT_SEC;
  public static double SENTINEL_GEOMETRY_EPSILON_IN;
  public static boolean CASABLANCA_ENABLE_DEPTH_PROTECTION;
  public static boolean CASABLANCA_ENABLE_SIDE_PROTECTION;
  public static double CASABLANCA_REPULSION_POWER;
  public static double CASABLANCA_REPULSION_DECEL_SAFETY_FACTOR;
  public static double CASABLANCA_DEPTH_SLOW_DOWN;
  public static double CASABLANCA_DEPTH_HARD_STOP;
  public static double CASABLANCA_SIDE_SLOW_DOWN;
  public static double CASABLANCA_SIDE_HARD_STOP;
  public static double CASABLANCA_LANE_BLEND_DISTANCE;
  public static double CASABLANCA_FRICTION_X;
  public static double CASABLANCA_FRICTION_Y;
  public static double CASABLANCA_FRICTION_ROT;
  public static boolean CASABLANCA_SMOOTHING_TELEOP_ENABLED;
  public static boolean CASABLANCA_SMOOTHING_AUTO_ENABLED;
  public static double CASABLANCA_SMOOTHING_FORWARD_INCREASE_TIME;
  public static double CASABLANCA_SMOOTHING_FORWARD_DECREASE_TIME;
  public static double CASABLANCA_SMOOTHING_STRAFE_INCREASE_TIME;
  public static double CASABLANCA_SMOOTHING_STRAFE_DECREASE_TIME;
  public static double CASABLANCA_SMOOTHING_TURN_INCREASE_TIME;
  public static double CASABLANCA_SMOOTHING_TURN_DECREASE_TIME;
  public static double CASABLANCA_SMOOTHING_MAX_LOOP_DT;
  public static boolean CASABLANCA_HEADING_LOCK_ENABLED;
  public static double CASABLANCA_HEADING_LOCK_PID_P;
  public static double CASABLANCA_HEADING_LOCK_PID_I;
  public static double CASABLANCA_HEADING_LOCK_PID_D;
  public static double CASABLANCA_HEADING_LOCK_KS;
  public static double CASABLANCA_HEADING_LOCK_INTENT_THRESHOLD;
  public static double CASABLANCA_HEADING_LOCK_KS_MOVING;
  public static double CASABLANCA_HEADING_LOCK_MOVING_SPEED_THRESHOLD;
  public static double CASABLANCA_HEADING_LOCK_MAX_POWER;
  public static double CASABLANCA_HEADING_LOCK_ERROR_DEADBAND_DEG;
  public static double CASABLANCA_HEADING_LOCK_SETTLE_RATE_DPS;
  public static Pose CASABLANCA_HEADING_TUNER_CALIBRATION_POSE;
  public static double CASABLANCA_HEADING_TUNER_STEP_DEGREES;
  public static double CASABLANCA_HEADING_TUNER_MAXIMUM_SWEEP_DEGREES;
  public static double CASABLANCA_HEADING_TUNER_POSITION_DRIFT_INCHES;
  public static double CASABLANCA_HEADING_TUNER_HEADING_TOLERANCE_DEGREES;
  public static double CASABLANCA_HEADING_TUNER_RATE_TOLERANCE_DPS;
  public static double CASABLANCA_HEADING_TUNER_OVERSHOOT_LIMIT_DEGREES;
  public static double CASABLANCA_HEADING_TUNER_SETTLE_SEC;
  public static double CASABLANCA_HEADING_TUNER_TRIAL_TIMEOUT_SEC;
  public static double CASABLANCA_HEADING_TUNER_STOPPED_SPEED_IPS;
  public static int CASABLANCA_HEADING_TUNER_SAMPLE_PERIOD_MS;
  public static double CASABLANCA_HEADING_TUNER_MAXIMUM_SAMPLE_GAP_SEC;
  public static double CASABLANCA_HEADING_TUNER_P_MIN;
  public static double CASABLANCA_HEADING_TUNER_P_MAX;
  public static int CASABLANCA_HEADING_TUNER_P_COUNT;
  public static double CASABLANCA_HEADING_TUNER_D_MIN;
  public static double CASABLANCA_HEADING_TUNER_D_MAX;
  public static int CASABLANCA_HEADING_TUNER_D_COUNT;
  public static String CASABLANCA_JUMP_TUNER_TRIAL_AXIS;
  public static double CASABLANCA_JUMP_TUNER_ROTATION_TRIAL_POWER;
  public static double CASABLANCA_JUMP_TUNER_DRIVE_ANGLE_DEGREES;
  public static double CASABLANCA_JUMP_TUNER_MAXIMUM_ANGLE_DEGREES;
  public static double CASABLANCA_JUMP_TUNER_ANGULAR_STOP_MARGIN_DEGREES;
  public static double CASABLANCA_JUMP_TUNER_ANGULAR_SPEED_BOUND_DPS;
  public static String CASABLANCA_JUMP_TUNER_IMU_NAME;
  public static String CASABLANCA_JUMP_TUNER_IMU_LOGO;
  public static String CASABLANCA_JUMP_TUNER_IMU_USB;
  public static Pose CASABLANCA_JUMP_TUNER_CALIBRATION_POSE;
  public static double CASABLANCA_JUMP_TUNER_TRIAL_POWER;
  public static double CASABLANCA_JUMP_TUNER_MAX_TRAVEL_INCHES;
  public static double CASABLANCA_JUMP_TUNER_DRIVE_DISTANCE_INCHES;
  public static double CASABLANCA_JUMP_TUNER_STOP_MARGIN_INCHES;
  public static double CASABLANCA_JUMP_TUNER_REACTION_TIME_SEC;
  public static double CASABLANCA_JUMP_TUNER_SPEED_BOUND_IPS;
  public static double CASABLANCA_JUMP_TUNER_BASELINE_SEC;
  public static double CASABLANCA_JUMP_TUNER_PEAK_HOLD_SEC;
  public static double CASABLANCA_JUMP_TUNER_SETTLE_SEC;
  public static double CASABLANCA_JUMP_TUNER_STOPPED_SPEED_IPS;
  public static double CASABLANCA_JUMP_TUNER_BASELINE_RATE_DPS;
  public static double CASABLANCA_JUMP_TUNER_JUMP_TILT_DEG;
  public static double CASABLANCA_JUMP_TUNER_JUMP_RATE_DPS;
  public static double CASABLANCA_JUMP_TUNER_JUMP_HOLD_SEC;
  public static double CASABLANCA_JUMP_TUNER_EMERGENCY_TILT_DEG;
  public static double CASABLANCA_JUMP_TUNER_IMU_STALE_SEC;
  public static double CASABLANCA_JUMP_TUNER_TRIAL_TIMEOUT_SEC;
  public static double CASABLANCA_JUMP_TUNER_STALL_TIMEOUT_SEC;
  public static double CASABLANCA_JUMP_TUNER_MINIMUM_PROGRESS_INCHES;
  public static double CASABLANCA_JUMP_TUNER_MINIMUM_PROGRESS_DEGREES;
  public static double CASABLANCA_JUMP_TUNER_HEADING_DRIFT_DEG;
  public static double CASABLANCA_JUMP_TUNER_LATERAL_DRIFT_INCHES;
  public static double CASABLANCA_JUMP_TUNER_POWER_TOLERANCE;
  public static int CASABLANCA_JUMP_TUNER_SAMPLE_PERIOD_MS;
  public static double CASABLANCA_JUMP_TUNER_MINIMUM_SLEW_TIME;
  public static double CASABLANCA_JUMP_TUNER_MAXIMUM_SLEW_TIME;
  public static double CASABLANCA_JUMP_TUNER_SEARCH_FACTOR;
  public static int CASABLANCA_JUMP_TUNER_CANDIDATE_COUNT;
  public static boolean SHOOTER_USE_FTC_PID;
  public static double SHOOTER_KS;
  public static double SHOOTER_NOMINAL_VOLTAGE;
  public static double SHOOTER_MAX_VOLTAGE_COMPENSATION;
  public static double SHOOTER_VOLTAGE_REFRESH_SEC;
  public static double SHOOTER_MAX_RPM;
  public static double SHOOTER_CONSTANT_RPM;
  public static double SHOOTER_MIN_TRANSFER_THRESHOLD;
  public static double SHOOTER_MAX_VELOCITY_THRESHOLD;
  public static double SHOOTER_FEED_RELEASE_THRESHOLD;
  public static double SHOOTER_FEED_INTAKE_POWER;
  public static double SHOOTER_FEED_TRANSFER_POWER;
  public static double SHOOTER_BALL_DETECTION_DIP_FRACTION;
  public static double SHOOTER_BALL_DETECTION_REBOUND_FRACTION;
  public static double SHOOTER_BALL_DETECTION_BASELINE_ALPHA;
  public static int SHOOTER_BALL_DETECTION_REFRACTORY_MS;
  public static double SHOOTER_PIDF_P;
  public static double SHOOTER_PIDF_I;
  public static double SHOOTER_PIDF_D;
  public static double SHOOTER_PIDF_F;
  public static double SHOOTER_MOTOR_PIDF_P;
  public static double SHOOTER_MOTOR_PIDF_I;
  public static double SHOOTER_MOTOR_PIDF_D;
  public static double SHOOTER_MOTOR_PIDF_F;
  public static double SHOOTER_INTEGRAL_BAND_TICKS;
  public static double SHOOTER_INTEGRAL_MAX_CONTRIBUTION;
  public static double SHOOTER_KICK_WHEEL_TAU_SEC;
  public static double SHOOTER_KICK_SENSOR_LAG_SEC;
  public static double SHOOTER_KICK_KICK_LAG_MARGIN_SEC;
  public static double SHOOTER_KICK_KICK_TRIGGER_FRACTION;
  public static double SHOOTER_KICK_KICK_SPEED_MARGIN;
  public static double SHOOTER_BALLISTICS_V0;
  public static double SHOOTER_BALLISTICS_K;
  public static double SHOOTER_BALLISTICS_MAGNUS_L;
  public static double SHOOTER_BALLISTICS_LAUNCH_HEIGHT_INCHES;
  public static double SHOOTER_BALLISTICS_G_INCHES_PER_SEC2;
  public static double SHOOTER_BALLISTICS_MIN_HOOD_ANGLE_DEG;
  public static double SHOOTER_BALLISTICS_MAX_HOOD_ANGLE_DEG;
  public static double SHOOTER_BALLISTICS_MIN_HOOD_SERVO_POS;
  public static double SHOOTER_BALLISTICS_MAX_HOOD_SERVO_POS;
  public static double SHOOTER_BALLISTICS_MAX_MOVING_SPEED_IPS;
  public static double SHOOTER_BALLISTICS_MIN_VALID_DISTANCE;
  public static double SHOOTER_BALLISTICS_MAX_VALID_DISTANCE;
  public static double SHOOTER_BALLISTICS_MAX_OFF_AXIS_DEG;
  public static double SHOOTER_BALLISTICS_LEAD_BIAS_GAIN;
  public static double SHOOTER_BALLISTICS_V0_REFERENCE_RPM;
  public static double SHOOTER_BALLISTICS_PREFERRED_SHOT_RPM;
  public static double SHOOTER_BALLISTICS_MAX_SHOT_RPM;
  public static double SHOOTER_BALLISTICS_V0_MARGIN_FRACTION;
  public static double SHOOTER_BALLISTICS_RPM_UPDATE_DEADBAND;
  public static double SHOOTER_BALLISTICS_FLIGHT_TIME_BASE_SEC;
  public static double SHOOTER_BALLISTICS_FLIGHT_TIME_SEC_PER_INCH;
  public static double SHOOTER_BALLISTICS_LEAD_MIN_SPEED_IPS;
  public static boolean SHOOTER_SHOT_TABLE_CALIBRATED;
  public static boolean SHOOTER_SHOT_TABLE_NECTAR_ENABLED;
  public static double[] SHOOTER_SHOT_TABLE_POLLEN_POINTS;
  public static double[] SHOOTER_SHOT_TABLE_NECTAR_POINTS;
  public static double[] AUTO_SHOT_TIME_POINTS;
  public static int AUTO_SHOOT_WAIT_MS;
  public static int AUTO_BALLS_PER_VOLLEY_COUNT;
  public static int AUTO_TIP_WAIT_MS;
  public static int AUTO_COLLECT_DWELL_MS;
  public static int AUTO_PARK_MARGIN_MS;
  public static int AUTO_PARK_RESERVE_MS;
  public static int AUTO_PARK_UPDATE_MS;
  public static double AUTO_TRANSFER_POWER;
  public static boolean AUTO_CELL_A_COLLECT_FLOWER;
  public static boolean AUTO_CELL_A_SECOND_VOLLEY;
  public static boolean AUTO_CELL_B_COLLECT_FLOWER;
  public static double TELEOP_MAX_SPEED;
  public static boolean TELEOP_FIELD_CENTRIC;
  public static double TELEOP_FIELD_CENTRIC_OFFSET_DEG;
  public static double TELEOP_INTAKE_POWER;
  public static double TELEOP_TRANSFER_POWER;
  public static double TELEOP_MANUAL_REV_POWER;
  public static double TELEOP_EJECT_POWER;
  public static boolean TELEOP_FLOWER_HOLD_ENABLED;
  public static int TELEOP_RUMBLE_MS;
  public static int TELEOP_PARK_ALERT_MARGIN_MS;
  public static int TELEOP_PARK_ALERT_UPDATE_MS;
  public static int TELEOP_PARK_ALERT_REPEAT_MS;
  public static Pose TELEOP_POSES_RED_START;
  public static Pose TELEOP_POSES_RED_PARK;
  public static Pose TELEOP_POSES_RED_SHOOT_A;
  public static Pose TELEOP_POSES_RED_SHOOT_B;
  public static Pose TELEOP_POSES_RED_FLOWER;
  public static Pose TELEOP_POSES_BLUE_START;
  public static Pose TELEOP_POSES_BLUE_PARK;
  public static Pose TELEOP_POSES_BLUE_SHOOT_A;
  public static Pose TELEOP_POSES_BLUE_SHOOT_B;
  public static Pose TELEOP_POSES_BLUE_FLOWER;
  public static boolean AGENT_BRIDGE_BRIDGE_ENABLED;
  public static double AGENT_BRIDGE_RECORD_RATE_HZ;
  public static double AGENT_BRIDGE_RECORD_MAX_ROWS;
  public static double AGENT_BRIDGE_CURRENT_EVERY_N_SAMPLES;
  public static double AGENT_BRIDGE_TELEMETRY_BUFFER_ENTRIES;
  public static int AGENT_BRIDGE_MAX_SCRIPT_MS;
  public static int AGENT_BRIDGE_SCRIPT_TICK_MS;
  public static int AGENT_BRIDGE_DEADMAN_MS;
  public static boolean AGENT_BRIDGE_FLIGHT_AUTO;
  public static double AGENT_BRIDGE_FLIGHT_POLL_HZ;
  public static int AGENT_BRIDGE_FLIGHT_FLUSH_MS;
  public static double AGENT_BRIDGE_FLIGHT_MAX_ROWS;
  public static double AGENT_BRIDGE_FLIGHT_KEEP;

  static {
    reload();
  }

  public static synchronized void reload() {
    ConfigLoader.reload();
    meta = ConfigLoader.load(Meta.class, "meta");
    field = ConfigLoader.load(Field.class, "field");
    hive = ConfigLoader.load(Hive.class, "hive");
    match = ConfigLoader.load(Match.class, "match");
    inventory = ConfigLoader.load(Inventory.class, "inventory");
    pedro = ConfigLoader.load(Pedro.class, "pedro");
    turret = ConfigLoader.load(Turret.class, "turret");
    vision = ConfigLoader.load(Vision.class, "vision");
    sentinel = ConfigLoader.load(Sentinel.class, "sentinel");
    casablanca = ConfigLoader.load(Casablanca.class, "casablanca");
    shooter = ConfigLoader.load(Shooter.class, "shooter");
    auto = ConfigLoader.load(Auto.class, "auto");
    teleop = ConfigLoader.load(Teleop.class, "teleop");
    agent_bridge = ConfigLoader.load(AgentBridge.class, "agent_bridge");
    META_SEASON = meta.season;
    FIELD_SIZE_IN = field.size_in;
    FIELD_HIVE_CENTER_TO_CENTER_IN = field.hive.center_to_center_in;
    FIELD_HIVE_PIVOT_HEIGHT_IN = field.hive.pivot_height_in;
    FIELD_HIVE_UP_OPENING_OFFSET_IN = field.hive.up_opening_offset_in;
    FIELD_HIVE_UP_OPENING_HEIGHT_IN = field.hive.up_opening_height_in;
    FIELD_HIVE_DOWN_OPENING_OFFSET_IN = field.hive.down_opening_offset_in;
    FIELD_HIVE_DOWN_OPENING_HEIGHT_IN = field.hive.down_opening_height_in;
    FIELD_HIVE_FRAME_WIDTH_IN = field.hive.frame_width_in;
    FIELD_HIVE_FRAME_DEPTH_IN = field.hive.frame_depth_in;
    FIELD_HIVE_BOTTOM_HEIGHT_IN = field.hive.bottom_height_in;
    FIELD_LOADING_ZONE_START_Y_IN = field.loading_zone.start_y_in;
    FIELD_LOADING_ZONE_WIDTH_IN = field.loading_zone.width_in;
    FIELD_LOADING_ZONE_DEPTH_IN = field.loading_zone.depth_in;
    FIELD_GARDEN_LENGTH_IN = field.garden.length_in;
    FIELD_GARDEN_DEPTH_IN = field.garden.depth_in;
    FIELD_FLOWER_WALL_OFFSET_IN = field.flower.wall_offset_in;
    FIELD_FLOWER_CORNER_OFFSET_IN = field.flower.corner_offset_in;
    HIVE_UP_HEIGHT_THRESHOLD_IN = hive.up_height_threshold_in;
    HIVE_CONFIRM_FRAMES_COUNT = hive.confirm_frames_count;
    HIVE_SETTLE_MS = hive.settle_ms;
    HIVE_MAX_OBSERVATION_AGE_MS = hive.max_observation_age_ms;
    MATCH_AUTO_MS = match.auto_ms;
    MATCH_TELEOP_MS = match.teleop_ms;
    MATCH_FLOWER_UNLOCK_REMAINING_MS = match.flower_unlock_remaining_ms;
    MATCH_FINAL_WARNING_REMAINING_MS = match.final_warning_remaining_ms;
    INVENTORY_CAPACITY_COUNT = inventory.capacity_count;
    INVENTORY_PRELOAD_COUNT = inventory.preload_count;
    INVENTORY_ENTRY_SENSOR_ENABLED = inventory.entry_sensor_enabled;
    INVENTORY_ENTRY_SENSOR_NAME = inventory.entry_sensor_name;
    INVENTORY_ENTRY_SENSOR_ACTIVE_LOW = inventory.entry_sensor_active_low;
    INVENTORY_ENTRY_DEBOUNCE_MS = inventory.entry_debounce_ms;
    PEDRO_LOCALIZATION_X_POD_OFFSET = pedro.localization.x_pod_offset;
    PEDRO_LOCALIZATION_Y_POD_OFFSET = pedro.localization.y_pod_offset;
    PEDRO_HEADING_PID_P = pedro.heading.pid.p;
    PEDRO_HEADING_PID_I = pedro.heading.pid.i;
    PEDRO_HEADING_PID_D = pedro.heading.pid.d;
    PEDRO_HEADING_KS = pedro.heading.ks;
    PEDRO_TRANSLATION_GAIN_THRESHOLD = pedro.translation.gain_threshold;
    PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_P = pedro.translation.forward.small_error_pid.p;
    PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_I = pedro.translation.forward.small_error_pid.i;
    PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_D = pedro.translation.forward.small_error_pid.d;
    PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_P = pedro.translation.forward.large_error_pid.p;
    PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_I = pedro.translation.forward.large_error_pid.i;
    PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_D = pedro.translation.forward.large_error_pid.d;
    PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_P = pedro.translation.strafe.small_error_pid.p;
    PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_I = pedro.translation.strafe.small_error_pid.i;
    PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_D = pedro.translation.strafe.small_error_pid.d;
    PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_P = pedro.translation.strafe.large_error_pid.p;
    PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_I = pedro.translation.strafe.large_error_pid.i;
    PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_D = pedro.translation.strafe.large_error_pid.d;
    PEDRO_FEEDFORWARD_COAST_KV = pedro.feedforward.coast_kv;
    PEDRO_FEEDFORWARD_BRAKE_KV = pedro.feedforward.brake_kv;
    PEDRO_BRAKING_FORWARD_LINEAR = pedro.braking.forward.linear;
    PEDRO_BRAKING_FORWARD_QUADRATIC = pedro.braking.forward.quadratic;
    PEDRO_BRAKING_STRAFE_LINEAR = pedro.braking.strafe.linear;
    PEDRO_BRAKING_STRAFE_QUADRATIC = pedro.braking.strafe.quadratic;
    PEDRO_BRAKING_HEADING_LINEAR = pedro.braking.heading.linear;
    PEDRO_BRAKING_HEADING_QUADRATIC = pedro.braking.heading.quadratic;
    PEDRO_MOTION_MAX_FORWARD_VELOCITY = pedro.motion.max_forward_velocity;
    PEDRO_MOTION_MAX_STRAFE_VELOCITY = pedro.motion.max_strafe_velocity;
    PEDRO_MOTION_NATURAL_FORWARD_DECELERATION = pedro.motion.natural_forward_deceleration;
    PEDRO_MOTION_NATURAL_STRAFE_DECELERATION = pedro.motion.natural_strafe_deceleration;
    PEDRO_MOTION_MAX_PATH_SPEED = pedro.motion.max_path_speed;
    TURRET_ENABLED = turret.enabled;
    TURRET_MAX_POWER_OUTPUT = turret.max_power_output;
    TURRET_SERVO_DIRECTION_INVERTED = turret.servo_direction_inverted;
    TURRET_KS_POSITIVE = turret.ks_positive;
    TURRET_KS_NEGATIVE = turret.ks_negative;
    TURRET_ORIENTATION_LOGO = turret.orientation.logo;
    TURRET_ORIENTATION_USB = turret.orientation.usb;
    TURRET_TRAVEL_MIN_ANGLE = turret.travel.min_angle;
    TURRET_TRAVEL_MAX_ANGLE = turret.travel.max_angle;
    TURRET_STALL_ENABLED = turret.stall.enabled;
    TURRET_STALL_POWER_THRESHOLD = turret.stall.power_threshold;
    TURRET_STALL_MIN_PROGRESS_DEG = turret.stall.min_progress_deg;
    TURRET_STALL_TIMEOUT_SEC = turret.stall.timeout_sec;
    TURRET_STALL_RETRY_AFTER_SEC = turret.stall.retry_after_sec;
    TURRET_RUNAWAY_ENABLED = turret.runaway.enabled;
    TURRET_RUNAWAY_DIVERGENCE_DEG = turret.runaway.divergence_deg;
    TURRET_PIDF_P = turret.pidf.p;
    TURRET_PIDF_I = turret.pidf.i;
    TURRET_PIDF_D = turret.pidf.d;
    TURRET_PIDF_F = turret.pidf.f;
    TURRET_KV = turret.kv;
    TURRET_FF_MIN_RATE_DPS = turret.ff_min_rate_dps;
    TURRET_PIVOT_FORWARD_IN = turret.pivot_forward_in;
    TURRET_PIVOT_LEFT_IN = turret.pivot_left_in;
    TURRET_INTEGRAL_BAND_DEG = turret.integral.band_deg;
    TURRET_INTEGRAL_LIMIT_POWER = turret.integral.limit_power;
    TURRET_TOLERANCE_MAX_DRIFT = turret.tolerance.max_drift;
    TURRET_TOLERANCE_NEAR_CUTOFF = turret.tolerance.near_cutoff;
    TURRET_TOLERANCE_NEAR_VAL = turret.tolerance.near_val;
    TURRET_TOLERANCE_MIN_DEG = turret.tolerance.min_deg;
    TURRET_TOLERANCE_MAX_DEG = turret.tolerance.max_deg;
    TURRET_TOLERANCE_EXIT_FACTOR = turret.tolerance.exit_factor;
    TURRET_ANALOG_ENCODER_ENABLED = turret.analog_encoder.enabled;
    TURRET_ANALOG_ENCODER_ZERO_VOLTAGE = turret.analog_encoder.zero_voltage;
    TURRET_ANALOG_ENCODER_DEGREES_PER_VOLT = turret.analog_encoder.degrees_per_volt;
    TURRET_ANALOG_ENCODER_MIN_VOLTAGE = turret.analog_encoder.min_voltage;
    TURRET_ANALOG_ENCODER_MAX_VOLTAGE = turret.analog_encoder.max_voltage;
    TURRET_ANALOG_ENCODER_FULL_SCALE_VOLTAGE = turret.analog_encoder.full_scale_voltage;
    TURRET_ANALOG_ENCODER_INVERTED = turret.analog_encoder.inverted;
    VISION_ENABLED = vision.enabled;
    VISION_CONTINUOUS = vision.continuous;
    VISION_CAMERA_NAME = vision.camera_name;
    VISION_DECIMATION = vision.decimation;
    VISION_CAMERA_FORWARD_IN = vision.camera.forward_in;
    VISION_CAMERA_LEFT_IN = vision.camera.left_in;
    VISION_CAMERA_HEIGHT_IN = vision.camera.height_in;
    VISION_CAMERA_YAW_DEG = vision.camera.yaw_deg;
    VISION_CAMERA_PITCH_UP_DEG = vision.camera.pitch_up_deg;
    VISION_RELOCALIZE_MAX_RANGE_IN = vision.relocalize.max_range_in;
    VISION_RELOCALIZE_MAX_CORRECTION_IN = vision.relocalize.max_correction_in;
    SENTINEL_ROBOT_WIDTH = sentinel.robot_width;
    SENTINEL_ROTATION_LOOKAHEAD_TIME = sentinel.rotation_lookahead_time;
    SENTINEL_PROTECT_FULL_HIVE_FOOTPRINT = sentinel.protect_full_hive_footprint;
    SENTINEL_FRAME_BASE_WIDTH_IN = sentinel.frame_base_width_in;
    SENTINEL_CENTER_LINE_MARGIN_IN = sentinel.center_line_margin_in;
    SENTINEL_WALL_CONTACT_TOLERANCE_IN = sentinel.wall_contact_tolerance_in;
    SENTINEL_PARK_LANE_MARGIN_IN = sentinel.park_lane_margin_in;
    SENTINEL_ROUTE_SAMPLE_STEP = sentinel.route_sample_step;
    SENTINEL_PARK_TURN_RATE_DPS = sentinel.park_turn_rate_dps;
    SENTINEL_PARK_SETTLE_PER_WAYPOINT_SEC = sentinel.park_settle_per_waypoint_sec;
    SENTINEL_GEOMETRY_EPSILON_IN = sentinel.geometry_epsilon_in;
    CASABLANCA_ENABLE_DEPTH_PROTECTION = casablanca.enable_depth_protection;
    CASABLANCA_ENABLE_SIDE_PROTECTION = casablanca.enable_side_protection;
    CASABLANCA_REPULSION_POWER = casablanca.repulsion.power;
    CASABLANCA_REPULSION_DECEL_SAFETY_FACTOR = casablanca.repulsion.decel_safety_factor;
    CASABLANCA_DEPTH_SLOW_DOWN = casablanca.depth.slow_down;
    CASABLANCA_DEPTH_HARD_STOP = casablanca.depth.hard_stop;
    CASABLANCA_SIDE_SLOW_DOWN = casablanca.side.slow_down;
    CASABLANCA_SIDE_HARD_STOP = casablanca.side.hard_stop;
    CASABLANCA_LANE_BLEND_DISTANCE = casablanca.lane_blend_distance;
    CASABLANCA_FRICTION_X = casablanca.friction.x;
    CASABLANCA_FRICTION_Y = casablanca.friction.y;
    CASABLANCA_FRICTION_ROT = casablanca.friction.rot;
    CASABLANCA_SMOOTHING_TELEOP_ENABLED = casablanca.smoothing.teleop_enabled;
    CASABLANCA_SMOOTHING_AUTO_ENABLED = casablanca.smoothing.auto_enabled;
    CASABLANCA_SMOOTHING_FORWARD_INCREASE_TIME = casablanca.smoothing.forward.increase_time;
    CASABLANCA_SMOOTHING_FORWARD_DECREASE_TIME = casablanca.smoothing.forward.decrease_time;
    CASABLANCA_SMOOTHING_STRAFE_INCREASE_TIME = casablanca.smoothing.strafe.increase_time;
    CASABLANCA_SMOOTHING_STRAFE_DECREASE_TIME = casablanca.smoothing.strafe.decrease_time;
    CASABLANCA_SMOOTHING_TURN_INCREASE_TIME = casablanca.smoothing.turn.increase_time;
    CASABLANCA_SMOOTHING_TURN_DECREASE_TIME = casablanca.smoothing.turn.decrease_time;
    CASABLANCA_SMOOTHING_MAX_LOOP_DT = casablanca.smoothing.max_loop_dt;
    CASABLANCA_HEADING_LOCK_ENABLED = casablanca.heading_lock.enabled;
    CASABLANCA_HEADING_LOCK_PID_P = casablanca.heading_lock.pid.p;
    CASABLANCA_HEADING_LOCK_PID_I = casablanca.heading_lock.pid.i;
    CASABLANCA_HEADING_LOCK_PID_D = casablanca.heading_lock.pid.d;
    CASABLANCA_HEADING_LOCK_KS = casablanca.heading_lock.ks;
    CASABLANCA_HEADING_LOCK_INTENT_THRESHOLD = casablanca.heading_lock.intent_threshold;
    CASABLANCA_HEADING_LOCK_KS_MOVING = casablanca.heading_lock.ks_moving;
    CASABLANCA_HEADING_LOCK_MOVING_SPEED_THRESHOLD = casablanca.heading_lock.moving_speed_threshold;
    CASABLANCA_HEADING_LOCK_MAX_POWER = casablanca.heading_lock.max_power;
    CASABLANCA_HEADING_LOCK_ERROR_DEADBAND_DEG = casablanca.heading_lock.error_deadband_deg;
    CASABLANCA_HEADING_LOCK_SETTLE_RATE_DPS = casablanca.heading_lock.settle_rate_dps;
    CASABLANCA_HEADING_TUNER_CALIBRATION_POSE = casablanca.heading_tuner.calibration_pose;
    CASABLANCA_HEADING_TUNER_STEP_DEGREES = casablanca.heading_tuner.step_degrees;
    CASABLANCA_HEADING_TUNER_MAXIMUM_SWEEP_DEGREES = casablanca.heading_tuner.maximum_sweep_degrees;
    CASABLANCA_HEADING_TUNER_POSITION_DRIFT_INCHES = casablanca.heading_tuner.position_drift_inches;
    CASABLANCA_HEADING_TUNER_HEADING_TOLERANCE_DEGREES =
        casablanca.heading_tuner.heading_tolerance_degrees;
    CASABLANCA_HEADING_TUNER_RATE_TOLERANCE_DPS = casablanca.heading_tuner.rate_tolerance_dps;
    CASABLANCA_HEADING_TUNER_OVERSHOOT_LIMIT_DEGREES =
        casablanca.heading_tuner.overshoot_limit_degrees;
    CASABLANCA_HEADING_TUNER_SETTLE_SEC = casablanca.heading_tuner.settle_sec;
    CASABLANCA_HEADING_TUNER_TRIAL_TIMEOUT_SEC = casablanca.heading_tuner.trial_timeout_sec;
    CASABLANCA_HEADING_TUNER_STOPPED_SPEED_IPS = casablanca.heading_tuner.stopped_speed_ips;
    CASABLANCA_HEADING_TUNER_SAMPLE_PERIOD_MS = casablanca.heading_tuner.sample_period_ms;
    CASABLANCA_HEADING_TUNER_MAXIMUM_SAMPLE_GAP_SEC =
        casablanca.heading_tuner.maximum_sample_gap_sec;
    CASABLANCA_HEADING_TUNER_P_MIN = casablanca.heading_tuner.p_min;
    CASABLANCA_HEADING_TUNER_P_MAX = casablanca.heading_tuner.p_max;
    CASABLANCA_HEADING_TUNER_P_COUNT = casablanca.heading_tuner.p_count;
    CASABLANCA_HEADING_TUNER_D_MIN = casablanca.heading_tuner.d_min;
    CASABLANCA_HEADING_TUNER_D_MAX = casablanca.heading_tuner.d_max;
    CASABLANCA_HEADING_TUNER_D_COUNT = casablanca.heading_tuner.d_count;
    CASABLANCA_JUMP_TUNER_TRIAL_AXIS = casablanca.jump_tuner.trial_axis;
    CASABLANCA_JUMP_TUNER_ROTATION_TRIAL_POWER = casablanca.jump_tuner.rotation_trial_power;
    CASABLANCA_JUMP_TUNER_DRIVE_ANGLE_DEGREES = casablanca.jump_tuner.drive_angle_degrees;
    CASABLANCA_JUMP_TUNER_MAXIMUM_ANGLE_DEGREES = casablanca.jump_tuner.maximum_angle_degrees;
    CASABLANCA_JUMP_TUNER_ANGULAR_STOP_MARGIN_DEGREES =
        casablanca.jump_tuner.angular_stop_margin_degrees;
    CASABLANCA_JUMP_TUNER_ANGULAR_SPEED_BOUND_DPS = casablanca.jump_tuner.angular_speed_bound_dps;
    CASABLANCA_JUMP_TUNER_IMU_NAME = casablanca.jump_tuner.imu_name;
    CASABLANCA_JUMP_TUNER_IMU_LOGO = casablanca.jump_tuner.imu_logo;
    CASABLANCA_JUMP_TUNER_IMU_USB = casablanca.jump_tuner.imu_usb;
    CASABLANCA_JUMP_TUNER_CALIBRATION_POSE = casablanca.jump_tuner.calibration_pose;
    CASABLANCA_JUMP_TUNER_TRIAL_POWER = casablanca.jump_tuner.trial_power;
    CASABLANCA_JUMP_TUNER_MAX_TRAVEL_INCHES = casablanca.jump_tuner.max_travel_inches;
    CASABLANCA_JUMP_TUNER_DRIVE_DISTANCE_INCHES = casablanca.jump_tuner.drive_distance_inches;
    CASABLANCA_JUMP_TUNER_STOP_MARGIN_INCHES = casablanca.jump_tuner.stop_margin_inches;
    CASABLANCA_JUMP_TUNER_REACTION_TIME_SEC = casablanca.jump_tuner.reaction_time_sec;
    CASABLANCA_JUMP_TUNER_SPEED_BOUND_IPS = casablanca.jump_tuner.speed_bound_ips;
    CASABLANCA_JUMP_TUNER_BASELINE_SEC = casablanca.jump_tuner.baseline_sec;
    CASABLANCA_JUMP_TUNER_PEAK_HOLD_SEC = casablanca.jump_tuner.peak_hold_sec;
    CASABLANCA_JUMP_TUNER_SETTLE_SEC = casablanca.jump_tuner.settle_sec;
    CASABLANCA_JUMP_TUNER_STOPPED_SPEED_IPS = casablanca.jump_tuner.stopped_speed_ips;
    CASABLANCA_JUMP_TUNER_BASELINE_RATE_DPS = casablanca.jump_tuner.baseline_rate_dps;
    CASABLANCA_JUMP_TUNER_JUMP_TILT_DEG = casablanca.jump_tuner.jump_tilt_deg;
    CASABLANCA_JUMP_TUNER_JUMP_RATE_DPS = casablanca.jump_tuner.jump_rate_dps;
    CASABLANCA_JUMP_TUNER_JUMP_HOLD_SEC = casablanca.jump_tuner.jump_hold_sec;
    CASABLANCA_JUMP_TUNER_EMERGENCY_TILT_DEG = casablanca.jump_tuner.emergency_tilt_deg;
    CASABLANCA_JUMP_TUNER_IMU_STALE_SEC = casablanca.jump_tuner.imu_stale_sec;
    CASABLANCA_JUMP_TUNER_TRIAL_TIMEOUT_SEC = casablanca.jump_tuner.trial_timeout_sec;
    CASABLANCA_JUMP_TUNER_STALL_TIMEOUT_SEC = casablanca.jump_tuner.stall_timeout_sec;
    CASABLANCA_JUMP_TUNER_MINIMUM_PROGRESS_INCHES = casablanca.jump_tuner.minimum_progress_inches;
    CASABLANCA_JUMP_TUNER_MINIMUM_PROGRESS_DEGREES = casablanca.jump_tuner.minimum_progress_degrees;
    CASABLANCA_JUMP_TUNER_HEADING_DRIFT_DEG = casablanca.jump_tuner.heading_drift_deg;
    CASABLANCA_JUMP_TUNER_LATERAL_DRIFT_INCHES = casablanca.jump_tuner.lateral_drift_inches;
    CASABLANCA_JUMP_TUNER_POWER_TOLERANCE = casablanca.jump_tuner.power_tolerance;
    CASABLANCA_JUMP_TUNER_SAMPLE_PERIOD_MS = casablanca.jump_tuner.sample_period_ms;
    CASABLANCA_JUMP_TUNER_MINIMUM_SLEW_TIME = casablanca.jump_tuner.minimum_slew_time;
    CASABLANCA_JUMP_TUNER_MAXIMUM_SLEW_TIME = casablanca.jump_tuner.maximum_slew_time;
    CASABLANCA_JUMP_TUNER_SEARCH_FACTOR = casablanca.jump_tuner.search_factor;
    CASABLANCA_JUMP_TUNER_CANDIDATE_COUNT = casablanca.jump_tuner.candidate_count;
    SHOOTER_USE_FTC_PID = shooter.use_ftc_pid;
    SHOOTER_KS = shooter.ks;
    SHOOTER_NOMINAL_VOLTAGE = shooter.nominal_voltage;
    SHOOTER_MAX_VOLTAGE_COMPENSATION = shooter.max_voltage_compensation;
    SHOOTER_VOLTAGE_REFRESH_SEC = shooter.voltage_refresh_sec;
    SHOOTER_MAX_RPM = shooter.max_rpm;
    SHOOTER_CONSTANT_RPM = shooter.constant_rpm;
    SHOOTER_MIN_TRANSFER_THRESHOLD = shooter.min_transfer_threshold;
    SHOOTER_MAX_VELOCITY_THRESHOLD = shooter.max_velocity_threshold;
    SHOOTER_FEED_RELEASE_THRESHOLD = shooter.feed_release_threshold;
    SHOOTER_FEED_INTAKE_POWER = shooter.feed_intake_power;
    SHOOTER_FEED_TRANSFER_POWER = shooter.feed_transfer_power;
    SHOOTER_BALL_DETECTION_DIP_FRACTION = shooter.ball_detection.dip_fraction;
    SHOOTER_BALL_DETECTION_REBOUND_FRACTION = shooter.ball_detection.rebound_fraction;
    SHOOTER_BALL_DETECTION_BASELINE_ALPHA = shooter.ball_detection.baseline_alpha;
    SHOOTER_BALL_DETECTION_REFRACTORY_MS = shooter.ball_detection.refractory_ms;
    SHOOTER_PIDF_P = shooter.pidf.p;
    SHOOTER_PIDF_I = shooter.pidf.i;
    SHOOTER_PIDF_D = shooter.pidf.d;
    SHOOTER_PIDF_F = shooter.pidf.f;
    SHOOTER_MOTOR_PIDF_P = shooter.motor_pidf.p;
    SHOOTER_MOTOR_PIDF_I = shooter.motor_pidf.i;
    SHOOTER_MOTOR_PIDF_D = shooter.motor_pidf.d;
    SHOOTER_MOTOR_PIDF_F = shooter.motor_pidf.f;
    SHOOTER_INTEGRAL_BAND_TICKS = shooter.integral.band_ticks;
    SHOOTER_INTEGRAL_MAX_CONTRIBUTION = shooter.integral.max_contribution;
    SHOOTER_KICK_WHEEL_TAU_SEC = shooter.kick.wheel_tau_sec;
    SHOOTER_KICK_SENSOR_LAG_SEC = shooter.kick.sensor_lag_sec;
    SHOOTER_KICK_KICK_LAG_MARGIN_SEC = shooter.kick.kick_lag_margin_sec;
    SHOOTER_KICK_KICK_TRIGGER_FRACTION = shooter.kick.kick_trigger_fraction;
    SHOOTER_KICK_KICK_SPEED_MARGIN = shooter.kick.kick_speed_margin;
    SHOOTER_BALLISTICS_V0 = shooter.ballistics.v0;
    SHOOTER_BALLISTICS_K = shooter.ballistics.k;
    SHOOTER_BALLISTICS_MAGNUS_L = shooter.ballistics.magnus_l;
    SHOOTER_BALLISTICS_LAUNCH_HEIGHT_INCHES = shooter.ballistics.launch_height_inches;
    SHOOTER_BALLISTICS_G_INCHES_PER_SEC2 = shooter.ballistics.g_inches_per_sec2;
    SHOOTER_BALLISTICS_MIN_HOOD_ANGLE_DEG = shooter.ballistics.min_hood_angle_deg;
    SHOOTER_BALLISTICS_MAX_HOOD_ANGLE_DEG = shooter.ballistics.max_hood_angle_deg;
    SHOOTER_BALLISTICS_MIN_HOOD_SERVO_POS = shooter.ballistics.min_hood_servo_pos;
    SHOOTER_BALLISTICS_MAX_HOOD_SERVO_POS = shooter.ballistics.max_hood_servo_pos;
    SHOOTER_BALLISTICS_MAX_MOVING_SPEED_IPS = shooter.ballistics.max_moving_speed_ips;
    SHOOTER_BALLISTICS_MIN_VALID_DISTANCE = shooter.ballistics.min_valid_distance;
    SHOOTER_BALLISTICS_MAX_VALID_DISTANCE = shooter.ballistics.max_valid_distance;
    SHOOTER_BALLISTICS_MAX_OFF_AXIS_DEG = shooter.ballistics.max_off_axis_deg;
    SHOOTER_BALLISTICS_LEAD_BIAS_GAIN = shooter.ballistics.lead_bias_gain;
    SHOOTER_BALLISTICS_V0_REFERENCE_RPM = shooter.ballistics.v0_reference_rpm;
    SHOOTER_BALLISTICS_PREFERRED_SHOT_RPM = shooter.ballistics.preferred_shot_rpm;
    SHOOTER_BALLISTICS_MAX_SHOT_RPM = shooter.ballistics.max_shot_rpm;
    SHOOTER_BALLISTICS_V0_MARGIN_FRACTION = shooter.ballistics.v0_margin_fraction;
    SHOOTER_BALLISTICS_RPM_UPDATE_DEADBAND = shooter.ballistics.rpm_update_deadband;
    SHOOTER_BALLISTICS_FLIGHT_TIME_BASE_SEC = shooter.ballistics.flight_time_base_sec;
    SHOOTER_BALLISTICS_FLIGHT_TIME_SEC_PER_INCH = shooter.ballistics.flight_time_sec_per_inch;
    SHOOTER_BALLISTICS_LEAD_MIN_SPEED_IPS = shooter.ballistics.lead_min_speed_ips;
    SHOOTER_SHOT_TABLE_CALIBRATED = shooter.shot_table.calibrated;
    SHOOTER_SHOT_TABLE_NECTAR_ENABLED = shooter.shot_table.nectar_enabled;
    SHOOTER_SHOT_TABLE_POLLEN_POINTS = shooter.shot_table.pollen_points;
    SHOOTER_SHOT_TABLE_NECTAR_POINTS = shooter.shot_table.nectar_points;
    AUTO_SHOT_TIME_POINTS = auto.shot_time_points;
    AUTO_SHOOT_WAIT_MS = auto.shoot_wait_ms;
    AUTO_BALLS_PER_VOLLEY_COUNT = auto.balls_per_volley_count;
    AUTO_TIP_WAIT_MS = auto.tip_wait_ms;
    AUTO_COLLECT_DWELL_MS = auto.collect_dwell_ms;
    AUTO_PARK_MARGIN_MS = auto.park_margin_ms;
    AUTO_PARK_RESERVE_MS = auto.park_reserve_ms;
    AUTO_PARK_UPDATE_MS = auto.park_update_ms;
    AUTO_TRANSFER_POWER = auto.transfer_power;
    AUTO_CELL_A_COLLECT_FLOWER = auto.cell_a_collect_flower;
    AUTO_CELL_A_SECOND_VOLLEY = auto.cell_a_second_volley;
    AUTO_CELL_B_COLLECT_FLOWER = auto.cell_b_collect_flower;
    TELEOP_MAX_SPEED = teleop.max_speed;
    TELEOP_FIELD_CENTRIC = teleop.field_centric;
    TELEOP_FIELD_CENTRIC_OFFSET_DEG = teleop.field_centric_offset_deg;
    TELEOP_INTAKE_POWER = teleop.intake_power;
    TELEOP_TRANSFER_POWER = teleop.transfer_power;
    TELEOP_MANUAL_REV_POWER = teleop.manual_rev_power;
    TELEOP_EJECT_POWER = teleop.eject_power;
    TELEOP_FLOWER_HOLD_ENABLED = teleop.flower_hold_enabled;
    TELEOP_RUMBLE_MS = teleop.rumble_ms;
    TELEOP_PARK_ALERT_MARGIN_MS = teleop.park_alert_margin_ms;
    TELEOP_PARK_ALERT_UPDATE_MS = teleop.park_alert_update_ms;
    TELEOP_PARK_ALERT_REPEAT_MS = teleop.park_alert_repeat_ms;
    TELEOP_POSES_RED_START = teleop.poses.red.start;
    TELEOP_POSES_RED_PARK = teleop.poses.red.park;
    TELEOP_POSES_RED_SHOOT_A = teleop.poses.red.shoot_a;
    TELEOP_POSES_RED_SHOOT_B = teleop.poses.red.shoot_b;
    TELEOP_POSES_RED_FLOWER = teleop.poses.red.flower;
    TELEOP_POSES_BLUE_START = teleop.poses.blue.start;
    TELEOP_POSES_BLUE_PARK = teleop.poses.blue.park;
    TELEOP_POSES_BLUE_SHOOT_A = teleop.poses.blue.shoot_a;
    TELEOP_POSES_BLUE_SHOOT_B = teleop.poses.blue.shoot_b;
    TELEOP_POSES_BLUE_FLOWER = teleop.poses.blue.flower;
    AGENT_BRIDGE_BRIDGE_ENABLED = agent_bridge.bridge_enabled;
    AGENT_BRIDGE_RECORD_RATE_HZ = agent_bridge.record_rate_hz;
    AGENT_BRIDGE_RECORD_MAX_ROWS = agent_bridge.record_max_rows;
    AGENT_BRIDGE_CURRENT_EVERY_N_SAMPLES = agent_bridge.current_every_n_samples;
    AGENT_BRIDGE_TELEMETRY_BUFFER_ENTRIES = agent_bridge.telemetry_buffer_entries;
    AGENT_BRIDGE_MAX_SCRIPT_MS = agent_bridge.max_script_ms;
    AGENT_BRIDGE_SCRIPT_TICK_MS = agent_bridge.script_tick_ms;
    AGENT_BRIDGE_DEADMAN_MS = agent_bridge.deadman_ms;
    AGENT_BRIDGE_FLIGHT_AUTO = agent_bridge.flight_auto;
    AGENT_BRIDGE_FLIGHT_POLL_HZ = agent_bridge.flight_poll_hz;
    AGENT_BRIDGE_FLIGHT_FLUSH_MS = agent_bridge.flight_flush_ms;
    AGENT_BRIDGE_FLIGHT_MAX_ROWS = agent_bridge.flight_max_rows;
    AGENT_BRIDGE_FLIGHT_KEEP = agent_bridge.flight_keep;
  }

  public static final class Meta {
    /**
     * Season tag. ConfigLoader ignores an ADB-pushed override whose meta.season differs from (or
     * lacks) this value, so last season's pushed config can never override this season's bundled
     * values.
     */
    public String season;
  }

  public static final class Field {
    /**
     * Inside-wall field width (inches). The field centre is size_in / 2; BLUE geometry and every
     * 'm' pose are the 180-degree rotation of RED about it. Manual nominal 144, official CAD
     * ~141.3. Minimum: 120.0 Maximum: 150.0
     */
    public double size_in;

    public static final class Hive {
      /**
       * Horizontal distance (inches) between the RED and BLUE HIVE pivots, along x (manual Figure
       * 9-10). Minimum: 0.0
       */
      public double center_to_center_in;

      /** HIVE pivot height above the TILES (inches, manual Figure 9-8). Minimum: 0.0 */
      public double pivot_height_in;

      /**
       * Horizontal distance (inches) from the HIVE pivot to an upward-facing CELL's opening centre,
       * toward the side the opening faces (CAD-derived). This point is the SDK AprilTag cluster
       * origin and the shot target. Minimum: 0.0
       */
      public double up_opening_offset_in;

      /**
       * Height (inches) of an upward-facing CELL's opening centre above the TILES (manual: opening
       * spans 53.5-65.6 in). Minimum: 0.0
       */
      public double up_opening_height_in;

      /**
       * Horizontal distance (inches) from the HIVE pivot to a downward-facing CELL's opening centre
       * (the up geometry rotated 60 degrees about the pivot). Minimum: 0.0
       */
      public double down_opening_offset_in;

      /**
       * Height (inches) of a downward-facing CELL's opening centre above the TILES. Minimum: 0.0
       */
      public double down_opening_height_in;

      /** HIVE frame base width (inches) along x (manual Figure 9-8). Minimum: 0.0 */
      public double frame_width_in;

      /** HIVE frame base depth (inches) along y (manual Figure 9-8). Minimum: 0.0 */
      public double frame_depth_in;

      /**
       * Lowest HIVE point above the TILES (inches). TU05 corrected V1's 25.5 in typo to 30.6 in; a
       * robot must stay below this to pass under the HIVE. Minimum: 0.0
       */
      public double bottom_height_in;
    }

    public Hive hive;

    public static final class LoadingZone {
      /**
       * RED LOADING ZONE start along the RED alliance wall (inches from the audience wall). BLUE is
       * the 180-degree rotation. Minimum: 0.0
       */
      public double start_y_in;

      /** LOADING ZONE length along its wall (inches, manual: ~23 in). Minimum: 0.0 */
      public double width_in;

      /** Depth of a zone measured out from its wall (inches): LOADING ZONE ~11 in. Minimum: 0.0 */
      public double depth_in;
    }

    public LoadingZone loading_zone;

    public static final class Garden {
      /**
       * GARDEN length along the audience/rear wall from the alliance corner (inches, manual: ~23
       * in). Minimum: 0.0
       */
      public double length_in;

      /** Depth of a zone measured out from its wall (inches): LOADING ZONE ~11 in. Minimum: 0.0 */
      public double depth_in;
    }

    public Garden garden;

    /**
     * Pose with the intake at an alliance-side FLOWER's bottom retrieval opening (G418: POLLEN may
     * only be removed from the bottom).
     */
    public static final class Flower {
      /**
       * Distance (inches) from a FLOWER's top-ring centre to the perimeter wall it is mounted on
       * (official field CAD: 2.65). Minimum: 0.0
       */
      public double wall_offset_in;

      /**
       * Distance (inches) from the alliance corner to each alliance-side FLOWER along its wall
       * (official field CAD: 23.39 in from the field centre line). Minimum: 0.0
       */
      public double corner_offset_in;
    }

    public Flower flower;
  }

  public static final class Hive {
    /**
     * A detected CELL whose opening centre is above this height (inches) is classified
     * upward-facing. Upward openings sit at ~59.6 in and downward ones at ~38.2 in. Minimum: 0.0
     */
    public double up_height_threshold_in;

    /**
     * Consecutive agreeing AprilTag observations required before the HIVE tracker accepts a TIP it
     * did not already expect. Minimum: 1.0
     */
    public int confirm_frames_count;

    /**
     * After a TIP is detected or entered, feeding is locked out for this long while the HIVE swings
     * and spilled SCORING ELEMENTS clear (manual 10.5.1: launching into a tipping HIVE can stop the
     * TIP). Minimum: 0.0
     */
    public int settle_ms;

    /**
     * AprilTag CELL observations older than this (milliseconds) are ignored by the HIVE tracker.
     * Minimum: 0.0
     */
    public int max_observation_age_ms;
  }

  public static final class Match {
    /** Length of AUTO (milliseconds, manual 10.4). Minimum: 0.0 */
    public int auto_ms;

    /** Length of TELEOP (milliseconds, manual 10.4). Minimum: 0.0 */
    public int teleop_ms;

    /**
     * TELEOP time remaining (milliseconds) at which NECTAR may enter FLOWERS (G410) and humans may
     * enter all remaining NECTAR (G426). Minimum: 0.0
     */
    public int flower_unlock_remaining_ms;

    /**
     * TELEOP time remaining (milliseconds) at which the park alert starts estimating the drive to
     * the park pose. Nothing is signalled at this moment; the alert fires later, at
     * park_alert_margin_ms before the estimated last moment. Minimum: 0.0
     */
    public int final_warning_remaining_ms;
  }

  public static final class Inventory {
    /**
     * Maximum SCORING ELEMENTS the robot may CONTROL (G407: 4). The intake stops itself when the
     * tracked inventory reaches this. Minimum: 1.0 Maximum: 4.0
     */
    public int capacity_count;

    /**
     * SCORING ELEMENTS loaded at match start (manual 10.3.4: exactly 4 POLLEN). Minimum: 0.0
     * Maximum: 4.0
     */
    public int preload_count;

    /**
     * True when an intake-entry beam break / proximity sensor is fitted. Without it the inventory
     * count is unknown in TELEOP and the intake cannot cap itself at capacity_count.
     */
    public boolean entry_sensor_enabled;

    /** Robot configuration name of the intake-entry DigitalChannel sensor. */
    public String entry_sensor_name;

    /**
     * True if the entry sensor reads false while a SCORING ELEMENT blocks it (most beam breaks).
     */
    public boolean entry_sensor_active_low;

    /**
     * Minimum time (milliseconds) the entry sensor must stay blocked before a SCORING ELEMENT is
     * counted. Minimum: 0.0
     */
    public int entry_debounce_ms;
  }

  public static final class Pedro {
    public static final class Localization {
      /** Forward odometry pod offset in inches, recovered from local Pedro 3 tuning. */
      public double x_pod_offset;

      /** Strafe odometry pod offset in inches, recovered from local Pedro 3 tuning. */
      public double y_pod_offset;
    }

    public Localization localization;

    public static final class Heading {
      /**
       * Heading feedback PID coefficients, bound as SDK PIDFCoefficients. Current Pedro 3 tuning is
       * proportional-only: keep I and D at zero unless separately tuned.
       */
      public com.qualcomm.robotcore.hardware.PIDFCoefficients pid;

      /**
       * Static friction feedforward magnitude. Pedro heading uses zero with the recovered
       * proportional-only tuning; separate from PID feedback. Minimum: 0.0 Maximum: 1.0
       */
      public double ks;
    }

    public Heading heading;

    public static final class Translation {
      /**
       * Absolute translation error in inches at which each axis switches to its large-error PID
       * gains. Minimum: 0.0
       */
      public double gain_threshold;

      public static final class Forward {
        /**
         * Translation PID coefficients below gain_threshold, bound as SDK PIDFCoefficients.
         * Recovered Pedro 3 tuning uses only P; I and D are zero.
         */
        public com.qualcomm.robotcore.hardware.PIDFCoefficients small_error_pid;

        /**
         * Translation PID coefficients at or above gain_threshold, bound as SDK PIDFCoefficients.
         * Recovered Pedro 3 tuning uses only P; I and D are zero.
         */
        public com.qualcomm.robotcore.hardware.PIDFCoefficients large_error_pid;
      }

      public Forward forward;

      public static final class Strafe {
        /**
         * Translation PID coefficients below gain_threshold, bound as SDK PIDFCoefficients.
         * Recovered Pedro 3 tuning uses only P; I and D are zero.
         */
        public com.qualcomm.robotcore.hardware.PIDFCoefficients small_error_pid;

        /**
         * Translation PID coefficients at or above gain_threshold, bound as SDK PIDFCoefficients.
         * Recovered Pedro 3 tuning uses only P; I and D are zero.
         */
        public com.qualcomm.robotcore.hardware.PIDFCoefficients large_error_pid;
      }

      public Strafe strafe;
    }

    public Translation translation;

    public static final class Feedforward {
      /** Foresight coast feedforward gain recovered from local Pedro 3 tuning. Minimum: 0.0 */
      public double coast_kv;

      /** Foresight brake feedforward gain recovered from local Pedro 3 tuning. Minimum: 0.0 */
      public double brake_kv;
    }

    public Feedforward feedforward;

    public static final class Braking {
      public static final class Forward {
        /**
         * Linear coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double linear;

        /**
         * Quadratic coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double quadratic;
      }

      public Forward forward;

      public static final class Strafe {
        /**
         * Linear coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double linear;

        /**
         * Quadratic coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double quadratic;
      }

      public Strafe strafe;

      public static final class Heading {
        /**
         * Linear coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double linear;

        /**
         * Quadratic coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double quadratic;
      }

      public Heading heading;
    }

    public Braking braking;

    public static final class Motion {
      /**
       * Forward velocity in inches per second recovered verbatim from local tuning; unusually low,
       * verify on robot. Minimum: 0.0
       */
      public double max_forward_velocity;

      /** Strafe velocity in inches per second recovered from local Pedro 3 tuning. Minimum: 0.0 */
      public double max_strafe_velocity;

      /**
       * Foresight natural forward deceleration in inches per second squared, recovered from local
       * Pedro 3 tuning. Minimum: 0.0
       */
      public double natural_forward_deceleration;

      /**
       * Foresight natural strafe deceleration in inches per second squared, recovered from local
       * Pedro 3 tuning. Minimum: 0.0
       */
      public double natural_strafe_deceleration;

      /**
       * Maximum Foresight path speed as a fraction of measured top speed. Explicit speed policy,
       * not equivalent to the old v2 path-completion threshold. Minimum: 0.0 Maximum: 1.0
       */
      public double max_path_speed;
    }

    public Motion motion;
  }

  public static final class Turret {
    /** Enable/disable this subsystem, controller, watchdog, or feature. */
    public boolean enabled;

    /**
     * Maximum power limit (0.0 to 1.0) commanded to the turret CRServo. Limits maximum turret
     * rotation speed. Minimum: 0.0 Maximum: 1.0
     */
    public double max_power_output;

    /**
     * Flips which CRServo direction counts as increasing turret angle. Set this (not the analog
     * encoder's 'inverted' flag) when the turret drives away from its target.
     */
    public boolean servo_direction_inverted;

    /**
     * Static friction feedforward (kS) magnitude applied when turning turret in positive direction
     * (error > 0). Minimum: 0.0 Maximum: 1.0
     */
    public double ks_positive;

    /**
     * Static friction feedforward (kS) magnitude applied when turning turret in negative direction
     * (error < 0). Minimum: 0.0 Maximum: 1.0
     */
    public double ks_negative;

    public static final class Orientation {
      /**
       * Mounting orientation direction of the hub's logo panel (e.g. RIGHT, LEFT, UP, DOWN,
       * FORWARD, BACKWARD).
       */
      public String logo;

      /** Mounting direction of the hub's USB ports. */
      public String usb;
    }

    public Orientation orientation;

    public static final class Travel {
      /**
       * Most negative turret angle (degrees, relative to the chassis) the turret is allowed to
       * reach. Minimum: -180.0 Maximum: 0.0
       */
      public double min_angle;

      /**
       * Most positive turret angle (degrees, relative to the chassis) the turret is allowed to
       * reach. Minimum: 0.0 Maximum: 180.0
       */
      public double max_angle;
    }

    public Travel travel;

    public static final class Stall {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * Commanded turret power above which the stall watchdog starts checking for motion. Below
       * this the servo may legitimately not move, so no stall is inferred. Minimum: 0.0 Maximum:
       * 1.0
       */
      public double power_threshold;

      /**
       * Turret motion (degrees) that counts as progress and resets the stall watchdog. Set above
       * the analog encoder's noise floor. Minimum: 0.0
       */
      public double min_progress_deg;

      /**
       * Seconds of commanded power with no turret motion before the stall watchdog latches a fault
       * and cuts power. Protects the turret from being driven into a hard stop when the encoder
       * reading is wrong. Minimum: 0.0
       */
      public double timeout_sec;

      /**
       * Seconds after a turret stall fault before power is allowed again (0 keeps the fault latched
       * until the OpMode restarts). A ball jam or a defender holding the turret must not disable
       * aiming for the rest of the match. Minimum: 0.0 Maximum: 10.0
       */
      public double retry_after_sec;
    }

    public Stall stall;

    public static final class Runaway {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * How far (degrees) the turret's aim error may grow beyond its best-so-far value before the
       * runaway watchdog cuts power. Catches an inverted control loop, where positive power moves
       * the turret away from its target, before it reaches a hard stop. Minimum: 0.0
       */
      public double divergence_deg;
    }

    public Runaway runaway;
    public com.qualcomm.robotcore.hardware.PIDFCoefficients pidf;

    /**
     * Turret velocity feedforward (power per degree/second): 1 / K, where K is the plant gain the
     * Turret Plant Calibration OpMode reports. Drives the turret at the rate chassis rotation and
     * translation demand, so it stops lagging the target. 0 disables. Minimum: 0.0 Maximum: 0.1
     */
    public double kv;

    /**
     * A settled turret follows the chassis-driven rate (kS sign(v) + kV v) only above this rate
     * (degrees/second); slower rates are odometry noise and would make it chatter. Minimum: 0.0
     */
    public double ff_min_rate_dps;

    /**
     * Turret rotation axis position forward of the robot centre (inches). Aim bearings, shot
     * distances and moving-shot lead are measured from this point.
     */
    public double pivot_forward_in;

    /** Turret rotation axis position left of the robot centre (inches). */
    public double pivot_left_in;

    public static final class Integral {
      /**
       * The turret I term only integrates while the aim error is within this many degrees. 30
       * covers the whole travel; narrow it only if the turret has symmetric, well-measured static
       * friction compensation (a narrow band left a 3 deg stall when ks under-compensates).
       * Minimum: 0.0
       */
      public double band_deg;

      /** Largest power (0 to 1) the turret I term may contribute. Minimum: 0.0 Maximum: 1.0 */
      public double limit_power;
    }

    public Integral integral;

    public static final class Tolerance {
      /**
       * Total lateral miss (inches) allowed at the target: the turret aim tolerance is
       * asin((max_drift / 2) / distance), clamped to [min_deg, max_deg]. Size it from the CELL
       * opening width seen from the shooting angle minus the ball diameter. Minimum: 0.0
       */
      public double max_drift;

      /**
       * Distance boundary (inches) below which the target is considered close, switching to static
       * tolerance. Minimum: 0.0
       */
      public double near_cutoff;

      /** Target alignment tolerance (degrees) when within the near cutoff range. Minimum: 0.0 */
      public double near_val;

      /**
       * Minimum dynamic alignment tolerance (degrees) allowed. Prevents the turret from hunting
       * indefinitely when very close. Minimum: 0.0
       */
      public double min_deg;

      /**
       * Maximum dynamic alignment tolerance (degrees). Caps the tolerance at short range. Minimum:
       * 0.0
       */
      public double max_deg;

      /**
       * Turret settle hysteresis: settled when the error is within tolerance, unsettled again only
       * past tolerance * exit_factor. Stops hunting at the tolerance edge and makes the aim gate
       * steady. 1 disables the hysteresis. Minimum: 1.0 Maximum: 5.0
       */
      public double exit_factor;
    }

    public Tolerance tolerance;

    public static final class AnalogEncoder {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * Analog voltage (V) corresponding to 0 degree turret angle position. Minimum: 0.0 Maximum:
       * 3.3
       */
      public double zero_voltage;

      /**
       * Scale factor (degrees/volt) converting analog input voltage delta to turret angle in
       * degrees.
       */
      public double degrees_per_volt;

      /** Minimum safe voltage limit for the analog encoder. Minimum: 0.0 Maximum: 3.3 */
      public double min_voltage;

      /** Maximum safe voltage limit for the analog encoder. Minimum: 0.0 Maximum: 3.3 */
      public double max_voltage;

      /**
       * Full electrical span (volts) of the turret potentiometer, i.e. the voltage at which it
       * wraps back to zero. Minimum: 0.0
       */
      public double full_scale_voltage;

      /** True if increasing turret angle produces decreasing analog voltage. */
      public boolean inverted;
    }

    public AnalogEncoder analog_encoder;
  }

  public static final class Vision {
    /** Enable/disable this subsystem, controller, watchdog, or feature. */
    public boolean enabled;

    /**
     * Stream the camera continuously so the HIVE tracker sees every TIP. False streams only while
     * the operator holds relocalize.
     */
    public boolean continuous;

    /** Robot configuration name of the webcam. */
    public String camera_name;

    /**
     * AprilTag decimation. Higher is faster with shorter detection range. Minimum: 1.0 Maximum: 6.0
     */
    public double decimation;

    public static final class Camera {
      /** Camera lens position forward of the robot centre (inches). */
      public double forward_in;

      /** Camera lens position left of the robot centre (inches). */
      public double left_in;

      /** Camera lens height above the TILES (inches). Minimum: 0.0 */
      public double height_in;

      /**
       * Camera yaw (degrees) from robot forward, counter-clockwise positive. Minimum: -180.0
       * Maximum: 180.0
       */
      public double yaw_deg;

      /**
       * Camera pitch (degrees) above horizontal. BIOBUZZ tags are on CELL bottoms 30-60 in up, so
       * this is positive. Minimum: -90.0 Maximum: 90.0
       */
      public double pitch_up_deg;
    }

    public Camera camera;

    public static final class Relocalize {
      /**
       * Ignore CELL detections farther than this (inches) for relocalization; pose error grows with
       * range. Minimum: 0.0
       */
      public double max_range_in;

      /**
       * Reject a vision relocalization that moves the pose farther than this (inches) from
       * odometry; that is a misidentified or moving CELL, not drift. Minimum: 0.0
       */
      public double max_correction_in;
    }

    public Relocalize relocalize;
  }

  public static final class Sentinel {
    /**
     * Total physical width of the robot (inches) used to build its square collision footprint for
     * HIVE-frame protection, LEAVE/PARK checks and the G402 alliance-side check. Minimum: 0.0
     */
    public double robot_width;

    /**
     * Time horizon (seconds) used to project future rotation based on live angular velocity
     * (|omega| * time). Seeded at 0.075s (derived from 0.45rad / assumed 6.0rad/s max omega);
     * calibrate on robot using Friction Calibration TeleOp. Minimum: 0.0
     */
    public double rotation_lookahead_time;

    /**
     * True protects the whole HIVE frame base rectangle (the robot never drives under the HIVE).
     * False protects only the two 2 in frame foot bars, for a robot built to pass under the HIVE;
     * the inward-leaning A-frame legs are not modelled, so verify leg clearance for the robot's
     * height.
     */
    public boolean protect_full_hive_footprint;

    /**
     * Width (inches) of each triangular HIVE frame base rail, used when protect_full_hive_footprint
     * is false. Minimum: 0.0
     */
    public double frame_base_width_in;

    /**
     * Margin (inches) the robot footprint must keep from the field centre line to count as fully on
     * its own side (G304.A / G402). Minimum: 0.0
     */
    public double center_line_margin_in;

    /**
     * Footprint distance from a perimeter wall (inches) that counts as touching for LEAVE. Minimum:
     * 0.0
     */
    public double wall_contact_tolerance_in;

    /**
     * Clearance (inches) between the HIVE frame and the robot's turning radius in the emergency
     * park lane. Minimum: 0.0
     */
    public double park_lane_margin_in;

    /**
     * Parameter increment for autonomous and assisted-drive path safety sampling. Smaller values
     * check more positions. Minimum: 0.001 Maximum: 0.02
     */
    public double route_sample_step;

    /**
     * Turn rate (degrees/second) assumed when estimating how long a park route takes. Lower is more
     * conservative. Minimum: 1.0
     */
    public double park_turn_rate_dps;

    /**
     * Time (seconds) added per waypoint when estimating how long a park route takes, for slowing
     * and settling at each corner. Minimum: 0.0
     */
    public double park_settle_per_waypoint_sec;

    /**
     * Numerical tolerance (inches) for footprint containment at exact field boundaries after
     * floating-point rotation. Minimum: 0.0 Maximum: 1.0E-6
     */
    public double geometry_epsilon_in;
  }

  public static final class Casablanca {
    /**
     * Enable/disable X-axis protection against driving into the HIVE frame. Safety policy
     * parameter; leave true outside of debugging.
     */
    public boolean enable_depth_protection;

    /**
     * Enable/disable Y-axis protection against driving into the HIVE frame. Safety policy
     * parameter; leave true outside of debugging.
     */
    public boolean enable_side_protection;

    public static final class Repulsion {
      /**
       * Motor power (0.0 to 1.0) commanded to push the robot back out when it is inside a protected
       * zone (the HIVE frame). Driver/safety policy parameter. Minimum: 0.0 Maximum: 1.0
       */
      public double power;

      /**
       * Safety margin factor (0.0 to 1.0) for predictive braking (e.g. 0.7 = 30% safety margin over
       * physics model prediction). Risk tolerance/policy parameter. Minimum: 0.0 Maximum: 1.0
       */
      public double decel_safety_factor;
    }

    public Repulsion repulsion;

    public static final class Depth {
      /**
       * Distance (inches) from a protected zone (the HIVE frame) at which proximity speed reduction
       * starts. Driver comfort / early-warning feel parameter. Minimum: 0.0
       */
      public double slow_down;

      /**
       * Distance (inches) from a protected zone (the HIVE frame) where drive power toward it is cut
       * to zero, so the robot cannot ram the HIVE (G417). Safety policy parameter. Minimum: 0.0
       */
      public double hard_stop;
    }

    public Depth depth;

    public static final class Side {
      /**
       * Distance (inches) from a protected zone (the HIVE frame) at which proximity speed reduction
       * starts. Driver comfort / early-warning feel parameter. Minimum: 0.0
       */
      public double slow_down;

      /**
       * Distance (inches) from a protected zone (the HIVE frame) where drive power toward it is cut
       * to zero, so the robot cannot ram the HIVE (G417). Safety policy parameter. Minimum: 0.0
       */
      public double hard_stop;
    }

    public Side side;

    /**
     * Distance (inches) over which safety slows downs are interpolated from 0% to 100% strength to
     * prevent jerky speed changes. Minimum: 0.0
     */
    public double lane_blend_distance;

    public static final class Friction {
      /** X value coordinate. */
      public double x;

      /** Y value coordinate. */
      public double y;

      /**
       * Minimum turn/yaw drive power (0.0 to 1.0) needed to overcome static wheel stiction.
       * Calibrated using Friction Calibration OpMode. Minimum: 0.0 Maximum: 1.0
       */
      public double rot;
    }

    public Friction friction;

    public static final class Smoothing {
      /**
       * Apply directional power slew to Casablanca TeleOp input. Per-instance commands may override
       * it.
       */
      public boolean teleop_enabled;

      /**
       * Apply directional slew to Pedro FOLLOW/HOLD output. False preserves native autonomous
       * output independently of TeleOp.
       */
      public boolean auto_enabled;

      public static final class Forward {
        /**
         * Seconds per unit increase in signed robot-axis power; 0 is unrestricted. Includes
         * acceleration in the positive direction and braking from negative power. Tune pod contact
         * for each axis. Minimum: 0.0
         */
        public double increase_time;

        /**
         * Seconds per unit decrease in signed robot-axis power; 0 is unrestricted. Includes braking
         * from positive power and acceleration in the negative direction. Minimum: 0.0
         */
        public double decrease_time;
      }

      public Forward forward;

      public static final class Strafe {
        /**
         * Seconds per unit increase in signed robot-axis power; 0 is unrestricted. Includes
         * acceleration in the positive direction and braking from negative power. Tune pod contact
         * for each axis. Minimum: 0.0
         */
        public double increase_time;

        /**
         * Seconds per unit decrease in signed robot-axis power; 0 is unrestricted. Includes braking
         * from positive power and acceleration in the negative direction. Minimum: 0.0
         */
        public double decrease_time;
      }

      public Strafe strafe;

      public static final class Turn {
        /**
         * Seconds per unit increase in signed robot-axis power; 0 is unrestricted. Includes
         * acceleration in the positive direction and braking from negative power. Tune pod contact
         * for each axis. Minimum: 0.0
         */
        public double increase_time;

        /**
         * Seconds per unit decrease in signed robot-axis power; 0 is unrestricted. Includes braking
         * from positive power and acceleration in the negative direction. Minimum: 0.0
         */
        public double decrease_time;
      }

      public Turn turn;

      /**
       * Maximum elapsed seconds used by the shared slew limiter per update; prevents a large power
       * step after a stalled loop. Minimum: 0.001 Maximum: 0.2
       */
      public double max_loop_dt;
    }

    public Smoothing smoothing;

    public static final class HeadingLock {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * Heading feedback PID coefficients, bound as SDK PIDFCoefficients. Current Pedro 3 tuning is
       * proportional-only: keep I and D at zero unless separately tuned.
       */
      public com.qualcomm.robotcore.hardware.PIDFCoefficients pid;

      /**
       * Static friction feedforward magnitude. Pedro heading uses zero with the recovered
       * proportional-only tuning; separate from PID feedback. Minimum: 0.0 Maximum: 1.0
       */
      public double ks;

      /**
       * Raw joystick turn threshold (0.0 to 1.0) above which driver active steering takes control
       * and disengages heading lock. Minimum: 0.0 Maximum: 1.0
       */
      public double intent_threshold;

      /**
       * Minimum rotational drive power (0.0 to 1.0) needed to overcome friction while the robot is
       * already translating. Minimum: 0.0 Maximum: 1.0
       */
      public double ks_moving;

      /**
       * Chassis translational speed threshold (inches/sec) above which full ks_moving friction
       * feedforward is applied. Minimum: 0.0
       */
      public double moving_speed_threshold;

      /**
       * Maximum motor power (0.0 to 1.0) the loop is allowed to command. Clamped to be completely
       * imperceptible. Minimum: 0.0 Maximum: 1.0
       */
      public double max_power;

      /**
       * Heading error (degrees) below which heading lock applies zero correction, to avoid
       * chattering/jitter from sensor noise while holding a heading at rest. Minimum: 0.0 Maximum:
       * 15.0
       */
      public double error_deadband_deg;

      /**
       * Rotation rate (degrees/sec) below which the robot counts as settled. Heading lock waits for
       * this before capturing the heading to hold, so it does not latch mid-coast and fight the
       * robot's own momentum. Minimum: 0.0
       */
      public double settle_rate_dps;
    }

    public HeadingLock heading_lock;

    public static final class HeadingTuner {
      /**
       * Actual field starting pose [x inches, y inches, heading degrees] for every manually
       * repositioned tuning trial; STRAFE trials start here turned 90 degrees. Collision protection
       * uses this pose, so it must leave HIVE protection inactive over the whole trial
       * (TunerPoseSafetyTest checks).
       */
      public Pose calibration_pose;

      /**
       * Signed heading step magnitude in each direction for TeleOp heading-lock tuning. Minimum:
       * 1.0 Maximum: 45.0
       */
      public double step_degrees;

      /**
       * Maximum deviation from each heading trial starting angle before an immediate stop. Minimum:
       * 1.0 Maximum: 90.0
       */
      public double maximum_sweep_degrees;

      /**
       * Maximum translation during an in-place heading trial before aborting. Minimum: 0.1 Maximum:
       * 12.0
       */
      public double position_drift_inches;

      /**
       * Required heading settling tolerance; must be at least the TeleOp heading-lock deadband.
       * Minimum: 0.1 Maximum: 10.0
       */
      public double heading_tolerance_degrees;

      /** Maximum absolute yaw rate for settled heading measurements. Minimum: 0.1 Maximum: 30.0 */
      public double rate_tolerance_dps;

      /**
       * Reject heading gains whose measured overshoot exceeds this limit. Minimum: 0.1 Maximum:
       * 20.0
       */
      public double overshoot_limit_degrees;

      /**
       * Stationary observation time after stopping; includes delayed chassis rocking in the jump
       * result. Minimum: 0.1 Maximum: 3.0
       */
      public double settle_sec;

      /**
       * Maximum duration of one jump-tuning trial including baseline and settling. Minimum: 0.5
       * Maximum: 30.0
       */
      public double trial_timeout_sec;

      /**
       * Linear velocity magnitude below which a jump-tuning trial is considered stationary.
       * Minimum: 0.01 Maximum: 5.0
       */
      public double stopped_speed_ips;

      /**
       * Delay between jump-tuner control-loop updates, in milliseconds. Minimum: 1.0 Maximum: 50.0
       */
      public int sample_period_ms;

      /**
       * Abort a heading trial after a control-loop delay above this duration. Minimum: 0.01
       * Maximum: 0.5
       */
      public double maximum_sample_gap_sec;

      /**
       * Lowest P gain tested by the TeleOp heading-lock step-response tuner (radians). Minimum: 0.0
       */
      public double p_min;

      /**
       * Highest P gain tested by the TeleOp heading-lock step-response tuner (radians). Minimum:
       * 0.0
       */
      public double p_max;

      /** Number of evenly spaced P candidates; 1 tests p_min only. Minimum: 1.0 Maximum: 10.0 */
      public int p_count;

      /**
       * Lowest D gain tested by the TeleOp heading-lock tuner using measured yaw rate. Minimum: 0.0
       */
      public double d_min;

      /**
       * Highest D gain tested by the TeleOp heading-lock tuner using measured yaw rate. Minimum:
       * 0.0
       */
      public double d_max;

      /** Number of evenly spaced D candidates; 1 tests d_min only. Minimum: 1.0 Maximum: 10.0 */
      public int d_count;
    }

    public HeadingTuner heading_tuner;

    public static final class JumpTuner {
      /**
       * Pod contact tuning axis: FORWARD, STRAFE, or TURN. Results apply only to that axis and
       * tested power; mixed inputs need robot validation.
       */
      public String trial_axis;

      /**
       * Requested yaw input for TURN pod contact trials, before friction compensation. Minimum:
       * 0.01 Maximum: 1.0
       */
      public double rotation_trial_power;

      /**
       * Powered angular travel before requesting a ramped stop during a TURN pod contact trial.
       * Minimum: 1.0 Maximum: 90.0
       */
      public double drive_angle_degrees;

      /**
       * Angular travel budget including stopping during a TURN pod contact trial. Minimum: 1.0
       * Maximum: 180.0
       */
      public double maximum_angle_degrees;

      /** Angular braking reserve within the TURN trial travel budget. Minimum: 0.1 Maximum: 45.0 */
      public double angular_stop_margin_degrees;

      /**
       * Conservative full-power yaw speed bound for TURN trial braking prediction, degrees per
       * second. Minimum: 1.0
       */
      public double angular_speed_bound_dps;

      /**
       * Hardware-map name of the Control Hub IMU used when Pinpoint firmware lacks pitch/roll.
       * Defaults to the FTC built-in IMU name. Pinpoint firmware 3 or newer is preferred
       * automatically.
       */
      public String imu_name;

      /** REV IMU logo-facing direction for the jump tuner; must match the physical mounting. */
      public String imu_logo;

      /** REV IMU USB-facing direction for the jump tuner; must match the physical mounting. */
      public String imu_usb;

      /**
       * Actual field starting pose [x inches, y inches, heading degrees] for every manually
       * repositioned tuning trial; STRAFE trials start here turned 90 degrees. Collision protection
       * uses this pose, so it must leave HIVE protection inactive over the whole trial
       * (TunerPoseSafetyTest checks).
       */
      public Pose calibration_pose;

      /**
       * Requested forward/reverse input power for jump tuning, before Casablanca friction
       * compensation. Results are validated only at this power. Minimum: 0.01 Maximum: 1.0
       */
      public double trial_power;

      /**
       * Maximum measured cumulative travel per trial, including stopping. Power is removed earlier
       * using a stopping reserve. The tuner rejects values above the requested 48-inch ceiling.
       * Minimum: 1.0 Maximum: 48.0
       */
      public double max_travel_inches;

      /**
       * Maximum powered travel before requesting a ramped stop; total travel including stopping
       * remains subject to max_travel_inches. Minimum: 0.1 Maximum: 48.0
       */
      public double drive_distance_inches;

      /**
       * Unpowered stopping reserve in inches inside the per-trial traversal limit. Minimum: 0.1
       * Maximum: 24.0
       */
      public double stop_margin_inches;

      /**
       * Loop/sensor reaction-time allowance used in jump-tuning stopping-room prediction. Minimum:
       * 0.001 Maximum: 1.0
       */
      public double reaction_time_sec;

      /**
       * Conservative forward speed estimate at full power used by jump-tuning stopping-room
       * prediction; seeded from the prior 75.6428 in/s measurement. Minimum: 0.1
       */
      public double speed_bound_ips;

      /**
       * Stationary IMU baseline duration before each jump-tuning trial. Minimum: 0.1 Maximum: 5.0
       */
      public double baseline_sec;

      /**
       * Required duration at the requested trial power before a passing trial can certify a
       * setting. Minimum: 0.01 Maximum: 1.0
       */
      public double peak_hold_sec;

      /**
       * Stationary observation time after stopping; includes delayed chassis rocking in the jump
       * result. Minimum: 0.1 Maximum: 3.0
       */
      public double settle_sec;

      /**
       * Linear velocity magnitude below which a jump-tuning trial is considered stationary.
       * Minimum: 0.01 Maximum: 5.0
       */
      public double stopped_speed_ips;

      /**
       * Maximum pitch/roll angular-rate magnitude while taking the stationary baseline. Minimum:
       * 0.1 Maximum: 30.0
       */
      public double baseline_rate_dps;

      /**
       * Diagnostic combined chassis tilt threshold. Does not establish 4-bar pod contact or reject
       * a contact-confirmed trial. Minimum: 0.1 Maximum: 20.0
       */
      public double jump_tilt_deg;

      /**
       * Diagnostic combined pitch/roll rate threshold. Pod contact is confirmed by the observer.
       * Minimum: 1.0 Maximum: 500.0
       */
      public double jump_rate_dps;

      /**
       * Duration for the diagnostic tilt/rate indicator; pod contact verdict comes from the
       * observer. Minimum: 0.001 Maximum: 1.0
       */
      public double jump_hold_sec;

      /**
       * Baseline-relative tilt that aborts the trial immediately; must exceed jump_tilt_deg.
       * Minimum: 0.2 Maximum: 30.0
       */
      public double emergency_tilt_deg;

      /**
       * Maximum time without a new IMU orientation sample before aborting the trial. Minimum: 0.01
       * Maximum: 1.0
       */
      public double imu_stale_sec;

      /**
       * Maximum duration of one jump-tuning trial including baseline and settling. Minimum: 0.5
       * Maximum: 30.0
       */
      public double trial_timeout_sec;

      /**
       * Maximum time without measurable travel while drive power is requested. Minimum: 0.1
       * Maximum: 5.0
       */
      public double stall_timeout_sec;

      /**
       * Travel increment that refreshes the jump-tuner stall watchdog. Minimum: 0.01 Maximum: 2.0
       */
      public double minimum_progress_inches;

      /**
       * Angular travel that refreshes the TURN pod-contact trial stall watchdog. Minimum: 0.1
       * Maximum: 10.0
       */
      public double minimum_progress_degrees;

      /**
       * Maximum heading deviation from the trial start before aborting. Minimum: 0.1 Maximum: 45.0
       */
      public double heading_drift_deg;

      /**
       * Maximum sideways deviation from the straight calibration lane before aborting. Minimum: 0.1
       * Maximum: 12.0
       */
      public double lateral_drift_inches;

      /**
       * Allowed difference from the requested power when checking trial excitation and zero output.
       * Minimum: 1.0E-4 Maximum: 0.1
       */
      public double power_tolerance;

      /**
       * Delay between jump-tuner control-loop updates, in milliseconds. Minimum: 1.0 Maximum: 50.0
       */
      public int sample_period_ms;

      /**
       * Fastest nonzero directional slew time tested after an unrestricted contact trial fails.
       * Minimum: 0.001 Maximum: 2.0
       */
      public double minimum_slew_time;

      /**
       * Slowest directional slew time considered when finding a contact-confirmed baseline.
       * Minimum: 0.001 Maximum: 3.0
       */
      public double maximum_slew_time;

      /**
       * Multiplicative step between candidate slew settings; must exceed one. Minimum: 1.01
       * Maximum: 2.0
       */
      public double search_factor;

      /**
       * Maximum number of candidate settings evaluated; a pass requires both forward and reverse
       * trials. Minimum: 1.0 Maximum: 50.0
       */
      public int candidate_count;
    }

    public JumpTuner jump_tuner;
  }

  public static final class Shooter {
    /**
     * When true, uses the REV firmware velocity PID on the motor (RUN_USING_ENCODER with
     * setVelocity, gains from shooter.motor_pidf). When false, uses the custom Pedro PIDF
     * controller, anti-windup integrator, and voltage compensation (RUN_WITHOUT_ENCODER, gains from
     * shooter.pidf).
     */
    public boolean use_ftc_pid;

    /**
     * Static friction feedforward magnitude. Pedro heading uses zero with the recovered
     * proportional-only tuning; separate from PID feedback. Minimum: 0.0 Maximum: 1.0
     */
    public double ks;

    /**
     * Bus voltage the flywheel feedforward (ks and pidf.f) was characterized at. Shooter.periodic()
     * scales its motor command by nominal_voltage divided by the measured bus voltage, so a sagging
     * pack still reaches the commanded wheel speed instead of quietly landing short and failing the
     * feed gate. Set this to the voltage shown while the Shooter Characterization OpMode is
     * running, not to a fresh-off-the-charger reading. Minimum: 1.0 Maximum: 20.0
     */
    public double nominal_voltage;

    /**
     * Largest factor the flywheel voltage compensation may scale its command by. Bounds the
     * correction so a brownout or a bad voltage reading cannot rail the flywheel; 1.4 covers a 12.0
     * V feedforward down to about 8.6 V. Minimum: 1.0 Maximum: 3.0
     */
    public double max_voltage_compensation;

    /**
     * How often the flywheel re-reads bus voltage. The hub's voltage is a separate ADC round trip
     * rather than part of the bulk read, so polling it every loop costs milliseconds per tick; a
     * pack does not sag meaningfully inside this interval. Minimum: 0.0 Maximum: 5.0
     */
    public double voltage_refresh_sec;

    /**
     * Flywheel velocity (DcMotorEx.setVelocity units: encoder ticks per second, despite the
     * historical _rpm name) that a target power of 1.0 maps to. Minimum: 0.0
     */
    public double max_rpm;

    /**
     * Fixed flywheel target (encoder ticks/s) for manual fixed-power shots and for holding the
     * flywheel spun up between volleys. Minimum: 0.0 Maximum: 3000.0
     */
    public double constant_rpm;

    /**
     * Flywheel speed ratio (0.0 to 1.0) required to arm a feed. Prevents launching before the
     * shooter is up to speed. Minimum: 0.0 Maximum: 1.0
     */
    public double min_transfer_threshold;

    /**
     * Flywheel velocity upper bound ratio (e.g. 1.03) to prevent overshooting before a feed is
     * armed. Applies only when arming, not while a feed is running, since a ball can only slow the
     * flywheel down. Minimum: 1.0 Maximum: 2.0
     */
    public double max_velocity_threshold;

    /**
     * Fraction of target flywheel speed a feed already in progress may sag to before the intake
     * cuts out. Must be below min_transfer_threshold: it is the lower half of a hysteresis band, so
     * each ring entering the flywheel does not drop the speed under the arming floor and stutter
     * the intake off and back on. Raise it toward min_transfer_threshold for stricter shot energy,
     * lower it if the intake still hitches while feeding. Minimum: 0.0 Maximum: 1.0
     */
    public double feed_release_threshold;

    /**
     * Front intake power (0.0 to 1.0) when feeding SCORING ELEMENTS to the shooter. Minimum: 0.0
     * Maximum: 1.0
     */
    public double feed_intake_power;

    /**
     * Transfer motor power (0.0 to 1.0) when feeding SCORING ELEMENTS to the shooter. Minimum: 0.0
     * Maximum: 1.0
     */
    public double feed_transfer_power;

    public static final class BallDetection {
      /**
       * How far flywheel speed must fall below its running reference, as a fraction, to count as a
       * ball leaving the shooter. Measured on this robot: real balls dip 12.2%-26.4%, the deepest
       * non-ball (the wheel coasting down after a shot) reaches 5.4%. Keep this between them; see
       * FlywheelDipDetector. Minimum: 0.0 Maximum: 1.0
       */
      public double dip_fraction;

      /**
       * How far flywheel speed must climb back up off a dip's own trough, as a fraction, before the
       * next ball can register. Must be strictly below dip_fraction. Ending a dip on a rebound off
       * the trough — not on a return to the pre-ball reference — is what lets balls that arrive
       * close together each get counted; see FlywheelDipDetector. Minimum: 0.0 Maximum: 1.0
       */
      public double rebound_fraction;

      /**
       * EMA weight per control loop for the ball detector's running reference speed. Small enough
       * that one ball cannot drag the reference down with it, large enough to track a flywheel
       * settling into a new cruise speed between shots. Minimum: 0.0 Maximum: 1.0
       */
      public double baseline_alpha;

      /**
       * Minimum spacing (milliseconds) between counted ball events. One ball produces a dip with a
       * ragged floor; without this, noise on the way back up would count as a second ball. Minimum:
       * 0.0
       */
      public int refractory_ms;
    }

    public BallDetection ball_detection;
    public com.qualcomm.robotcore.hardware.PIDFCoefficients pidf;

    /**
     * REV firmware velocity PIDF (p, i, d, f) applied directly to the flywheel motors via
     * DcMotorEx.setPIDFCoefficients when shooter.use_ftc_pid is true. Distinct from shooter.pidf,
     * which only feeds the software PIDFController used when use_ftc_pid is false and is never sent
     * to the motor. Keep i at 0 here: a live integral on the motor firmware winds up during a
     * feed's velocity sag and overshoots the setpoint once the ball leaves, which is what stalled
     * the third shot of a magazine.
     */
    public com.qualcomm.robotcore.hardware.PIDFCoefficients motor_pidf;

    public static final class Integral {
      /**
       * Flywheel velocity error (ticks/s) within which the integral term accumulates. Outside this
       * band the integrator holds its value, so spin-up and ball-feed transients never charge it.
       * Minimum: 0.0 Maximum: 2000.0
       */
      public double band_ticks;

      /**
       * Maximum absolute motor power the flywheel integral term may contribute. Bounds windup
       * independently of the integral gain. Minimum: 0.0 Maximum: 1.0
       */
      public double max_contribution;
    }

    public Integral integral;

    public static final class Kick {
      /**
       * Flywheel mechanical time constant (seconds): time to cover 63% of a power step, measured by
       * Shooter Characterization's step test. Used only by the recovery kick's model; 0 disables
       * the kick. Minimum: 0.0 Maximum: 5.0
       */
      public double wheel_tau_sec;

      /**
       * Dead time (seconds) between a flywheel power change and the first change in the measured
       * velocity, from Shooter Characterization's step test. The recovery kick runs its model
       * forward over this long so it stops on the wheel's speed now, not the reading's. Minimum:
       * 0.0 Maximum: 0.3
       */
      public double sensor_lag_sec;

      /**
       * How far shooter.kick.sensor_lag_sec may be short (the step test fits it to within 7.5 ms).
       * While kicking, the model runs over the lag plus this, which predicts an accelerating wheel
       * faster and so stops the kick early rather than late. Minimum: 0.0 Maximum: 0.1
       */
      public double kick_lag_margin_sec;

      /**
       * The recovery kick starts once the measured flywheel speed is this fraction below target (a
       * ball hit or spin-up). Match shooter.ball_detection.dip_fraction so every counted ball also
       * kicks. Minimum: 0.0 Maximum: 0.5
       */
      public double kick_trigger_fraction;

      /**
       * The recovery kick asks only for the power that would land a wheel this many times faster
       * than its model on target, so a model error costs a little speed rather than an overshoot.
       * 1.4 covers a 25% wheel_tau_sec error times the 10% kV error FlywheelClosedLoopProperties
       * allows (1.25 x 1.1). Minimum: 1.0 Maximum: 3.0
       */
      public double kick_speed_margin;
    }

    public Kick kick;

    public static final class Ballistics {
      /** Fitted initial ring exit velocity (in/s) at constant RPM. Minimum: 0.0 */
      public double v0;

      /**
       * Fitted lumped quadratic drag coefficient (1/in) for 2D trajectory integration. Minimum: 0.0
       */
      public double k;

      /**
       * Fitted lumped Magnus lift coefficient (1/s) for 2D backspin trajectory integration.
       * Minimum: 0.0
       */
      public double magnus_l;

      /**
       * Ball release height (inches) above the TILES. The target height is not configured here: it
       * is field.hive.up_opening_height_in, the upward CELL opening centre. Minimum: 0.0
       */
      public double launch_height_inches;

      /** Gravitational acceleration constant (in/s²). Minimum: 0.0 */
      public double g_inches_per_sec2;

      /** Minimum physical hood elevation angle (degrees). Minimum: 0.0 Maximum: 90.0 */
      public double min_hood_angle_deg;

      /** Maximum physical hood elevation angle (degrees). Minimum: 0.0 Maximum: 90.0 */
      public double max_hood_angle_deg;

      /**
       * Continuous servo position (0.0 to 1.0) corresponding to min_hood_angle_deg, the flattest
       * arc. The hood linkage is reversed, so this is numerically GREATER than max_hood_servo_pos;
       * that is expected and the angle mapping handles it. Do not 'fix' the ordering. Minimum: 0.0
       * Maximum: 1.0
       */
      public double min_hood_servo_pos;

      /**
       * Continuous servo position (0.0 to 1.0) corresponding to max_hood_angle_deg, the steepest
       * arc. Servo 0.0 is as tall as the arc gets, so this is normally the smaller number of the
       * pair. Minimum: 0.0 Maximum: 1.0
       */
      public double max_hood_servo_pos;

      /** Maximum allowed robot translation speed (in/s) for valid moving shots. Minimum: 0.0 */
      public double max_moving_speed_ips;

      /**
       * Minimum horizontal distance (inches) from the robot to the target CELL opening centre for a
       * valid solution. Minimum: 0.0
       */
      public double min_valid_distance;

      /**
       * Maximum horizontal distance (inches) from the robot to the target CELL opening centre for a
       * valid solution. Minimum: 0.0
       */
      public double max_valid_distance;

      /**
       * Largest angle (degrees) between the target CELL opening's outward direction and the line to
       * the robot for a valid shot. The opening faces outward and 30 degrees up, so shots from
       * behind or far to the side cannot enter it. Minimum: 0.0 Maximum: 90.0
       */
      public double max_off_axis_deg;

      /** Empirical calibration multiplier for virtual lead target calculation. Minimum: 0.0 */
      public double lead_bias_gain;

      /**
       * Flywheel setpoint the fitted v0 exit velocity was measured at. Exit speed and flywheel
       * setpoint convert through v0 / v0_reference_rpm, which is how the solver turns a required
       * exit speed into an RPM. Must match the RPM the ballistics calibration actually ran at
       * (constant_rpm), or every solved RPM is scaled wrong. Minimum: 1.0 Maximum: 3000.0
       */
      public double v0_reference_rpm;

      /**
       * Lower bound (encoder ticks/s) the solved flywheel setpoint is clamped to. Raise it if shots
       * land weak across the board. Minimum: 0.0 Maximum: 3000.0
       */
      public double preferred_shot_rpm;

      /**
       * Highest flywheel setpoint the RPM solver may command, for distances past the hood's
       * authority at preferred_shot_rpm. A target that cannot reach goal height even at this speed
       * is reported as an invalid solution instead of being silently under-shot. Minimum: 0.0
       * Maximum: 3000.0
       */
      public double max_shot_rpm;

      /**
       * Headroom above the slowest exit speed that still reaches the goal (0.12 = 12% faster),
       * applied only to long shots that had to raise RPM past preferred_shot_rpm. Keeps those
       * solutions off the boundary where the flat and lofted trajectory branches merge; larger
       * values give a steeper, more forgiving entry angle at the cost of a hotter shot. Minimum:
       * 0.0 Maximum: 1.0
       */
      public double v0_margin_fraction;

      /**
       * Minimum change in solved RPM (ticks/s) before the flywheel setpoint is re-commanded. Every
       * setpoint write discards the flywheel's accumulated anti-windup integral, so without a
       * deadband a continuously drifting target would keep the integrator reset and the flywheel
       * permanently short of speed. Minimum: 0.0
       */
      public double rpm_update_deadband;

      /**
       * Time-of-flight intercept (seconds): a BIOBUZZ lob spends most of its flight rising ~45 in
       * to the CELL, so flight time is base + per_inch * range rather than proportional to range.
       * Used only to lead a moving shot. Minimum: 0.0 Maximum: 2.0
       */
      public double flight_time_base_sec;

      /**
       * Additional time of flight per inch of horizontal range (seconds/inch), used only to lead a
       * moving shot. Minimum: 0.0 Maximum: 0.1
       */
      public double flight_time_sec_per_inch;

      /**
       * Robot speed below which moving-shot lead is skipped entirely. A robot holding position
       * still reports a small noisy velocity, and lead turns that noise into aim-azimuth jitter
       * larger than the turret's settle tolerance, so the turret hunts instead of settling. Leading
       * a nearly-stationary robot gains nothing. Minimum: 0.0 Maximum: 100.0
       */
      public double lead_min_speed_ips;
    }

    public Ballistics ballistics;

    public static final class ShotTable {
      /**
       * False marks the shot tables as placeholders: the automatic feed gate refuses to fire. Set
       * true only after the Ballistics Calibration OpMode has produced real BIOBUZZ rows.
       */
      public boolean calibrated;

      /**
       * False when the robot cannot handle NECTAR: the shot controller stays on the POLLEN table
       * and the TeleOp and Ballistics Calibration ball-type toggles do nothing. True allows
       * selecting the NECTAR table.
       */
      public boolean nectar_enabled;

      /**
       * POLLEN shot table as flat distance_in, hood_servo, flywheel_ticks_per_sec triples from the
       * Ballistics Calibration OpMode. Distances must strictly increase.
       */
      public double[] pollen_points;

      /**
       * NECTAR shot table as flat distance_in, hood_servo, flywheel_ticks_per_sec triples. NECTAR
       * is 3.6 in and heavier than 2.8 in POLLEN, so it needs its own calibration. Distances must
       * strictly increase.
       */
      public double[] nectar_points;
    }

    public ShotTable shot_table;
  }

  public static final class Auto {
    /**
     * Measured autonomous shoot windows as flat distance_in, shoot_window_ms pairs, written by the
     * Shot Timing Tuner OpMode in exactly this format. Distances must strictly increase;
     * interpolated and clamped at the ends. A volley ends when auto.balls_per_volley_count balls
     * are counted OR this window expires; the window is the jam safety net. auto.shoot_wait_ms is
     * the fallback when this table is empty.
     */
    public double[] shot_time_points;

    /**
     * Fallback autonomous shoot window (milliseconds), used only when auto.shot_time_points is
     * empty. Minimum: 0.0
     */
    public int shoot_wait_ms;

    /**
     * SCORING ELEMENTS expected per autonomous volley. A volley ends when this many shots are
     * counted by shooter.ball_detection or the shot window expires. Minimum: 1.0 Maximum: 4.0
     */
    public int balls_per_volley_count;

    /**
     * Autonomous wait (milliseconds) for the target CELL to be seen upward-facing before giving up
     * on it. The robot never launches at a downward-facing CELL (G417). Minimum: 0.0
     */
    public int tip_wait_ms;

    /**
     * Autonomous time (milliseconds) spent at a FLOWER retrieval opening with the intake running.
     * Minimum: 0.0
     */
    public int collect_dwell_ms;

    /**
     * AUTO park deadline margin (milliseconds): the routine is cancelled and the robot parks once
     * the time left is no more than the estimated drive to the park pose (stopping, then the
     * HIVE-avoiding route) plus this. Minimum: 0.0
     */
    public int park_margin_ms;

    /**
     * Latest AUTO park start (milliseconds before the end): the robot parks at this point even if
     * the estimated drive plus park_margin_ms is shorter, or when no estimate exists. Minimum: 0.0
     */
    public int park_reserve_ms;

    /**
     * How often (milliseconds) AUTO re-plans the park route and re-estimates the drive time for the
     * park deadline. Minimum: 20.0
     */
    public int park_update_ms;

    /**
     * Transfer motor power (0.0 to 1.0) while the intake is collecting (not feeding the shooter).
     * Minimum: 0.0 Maximum: 1.0
     */
    public double transfer_power;

    /**
     * CELL A routine: collect POLLEN from the alliance's side-wall FLOWER after the first volley.
     */
    public boolean cell_a_collect_flower;

    /**
     * CELL A routine: drive around the HIVE on the FLOWER-side lane and launch a second volley into
     * CELL B once it is upward-facing. Without cell_a_collect_flower this fires only POLLEN left
     * over when the TIP stopped the first volley.
     */
    public boolean cell_a_second_volley;

    /**
     * CELL B routine: collect POLLEN from the alliance's rear-wall FLOWER and launch a second
     * volley.
     */
    public boolean cell_b_collect_flower;
  }

  public static final class Teleop {
    /**
     * Maximum speed scaler (0.0 to 1.0) applied to chassis manual TeleOp drive inputs. Minimum: 0.0
     * Maximum: 1.0
     */
    public double max_speed;

    /**
     * When true, driver translation sticks are interpreted in the field frame (stick direction is a
     * fixed field direction regardless of robot heading). When false, sticks are robot-relative.
     */
    public boolean field_centric;

    /**
     * Field heading (degrees) that a fully-forward driver stick should drive toward when
     * field_centric is true. 90 means stick-forward drives along field +Y, i.e. away from the
     * driver wall at y=0. Set this to whatever direction is 'away' from where the drivers
     * physically stand. Minimum: -360.0 Maximum: 360.0
     */
    public double field_centric_offset_deg;

    /**
     * Target motor power (0.0 to 1.0) for the intake rollers in TeleOp. Minimum: 0.0 Maximum: 1.0
     */
    public double intake_power;

    /**
     * Transfer motor power (0.0 to 1.0) while the intake is collecting (not feeding the shooter).
     * Minimum: 0.0 Maximum: 1.0
     */
    public double transfer_power;

    /**
     * Target flywheel power ratio (0.0 to 1.0) for manual rev command in TeleOp. Minimum: 0.0
     * Maximum: 1.0
     */
    public double manual_rev_power;

    /**
     * Intake power (negative = reverse) for the operator eject button, used to spit out a fifth
     * SCORING ELEMENT (G407) or opponent NECTAR (G408). Minimum: -1.0 Maximum: 0.0
     */
    public double eject_power;

    /**
     * TeleOp: driver dpad left drives to and holds the FLOWER pickup pose. False when the robot
     * cannot take POLLEN from a FLOWER; the button then does nothing.
     */
    public boolean flower_hold_enabled;

    /** Gamepad rumble length (milliseconds) for match-time cues. Minimum: 0.0 */
    public int rumble_ms;

    /**
     * TELEOP park alert: fires once the time left is no more than the estimated drive to the park
     * pose plus this many milliseconds, and repeats until the robot is PARKED. Minimum: 0.0
     */
    public int park_alert_margin_ms;

    /**
     * How often (milliseconds) the TELEOP park alert re-plans the route and re-estimates the drive
     * time. Minimum: 20.0
     */
    public int park_alert_update_ms;

    /**
     * Period (milliseconds) of the TELEOP park alert's rumble, gamepad light flash and Driver
     * Station voice while it is active. Minimum: 200.0
     */
    public int park_alert_repeat_ms;

    public static final class Poses {
      public static final class Red {
        /**
         * Starting pose: touching the perimeter wall, fully on the alliance's side, not in the
         * LOADING ZONE and not touching a FLOWER (G304).
         */
        public Pose start;

        /**
         * Parking pose: footprint partially inside the alliance LOADING ZONE but clear of the
         * perimeter wall, so the robot earns both LEAVE and PARK at the end of AUTO.
         */
        public Pose park;

        /** Pose facing CELL A's opening from the side it opens toward. */
        public Pose shoot_a;

        /** Pose facing CELL B's opening from the side it opens toward. */
        public Pose shoot_b;

        /**
         * Pose with the intake at an alliance-side FLOWER's bottom retrieval opening (G418: POLLEN
         * may only be removed from the bottom).
         */
        public Pose flower;
      }

      public Red red;

      public static final class Blue {
        /**
         * Starting pose: touching the perimeter wall, fully on the alliance's side, not in the
         * LOADING ZONE and not touching a FLOWER (G304).
         */
        public Pose start;

        /**
         * Parking pose: footprint partially inside the alliance LOADING ZONE but clear of the
         * perimeter wall, so the robot earns both LEAVE and PARK at the end of AUTO.
         */
        public Pose park;

        /** Pose facing CELL A's opening from the side it opens toward. */
        public Pose shoot_a;

        /** Pose facing CELL B's opening from the side it opens toward. */
        public Pose shoot_b;

        /**
         * Pose with the intake at an alliance-side FLOWER's bottom retrieval opening (G418: POLLEN
         * may only be removed from the bottom).
         */
        public Pose flower;
      }

      public Blue blue;
    }

    public Poses poses;
  }

  public static final class AgentBridge {
    /**
     * Serve the agent bridge's /agent/ endpoints. False: every endpoint except /agent/status
     * answers 403. Turn off before competitions.
     */
    public boolean bridge_enabled;

    /**
     * Hardware recorder sample rate in Hz. Each sample sends one bulk-input command per hub,
     * independent of the OpMode's bulk cache, so higher rates add hub traffic the OpMode competes
     * with. Minimum: 1.0 Maximum: 200.0
     */
    public double record_rate_hz;

    /**
     * Hardware recorder capacity in samples; later samples are dropped and the recording marked
     * full. Memory is about 4 bytes per column per row. Minimum: 100.0 Maximum: 200000.0
     */
    public double record_max_rows;

    /**
     * Read every motor's current (one hub command per motor) on every Nth recorder sample; columns
     * hold the last reading in between. 0 never reads currents. Minimum: 0.0 Maximum: 1000.0
     */
    public double current_every_n_samples;

    /**
     * Number of OpMode telemetry updates the agent bridge keeps for /agent/telemetry. Minimum: 10.0
     * Maximum: 100000.0
     */
    public double telemetry_buffer_entries;

    /**
     * Longest gamepad script, in milliseconds, the agent bridge accepts. Minimum: 100.0 Maximum:
     * 600000.0
     */
    public int max_script_ms;

    /**
     * Period in milliseconds at which the agent bridge advances a running gamepad script: the
     * timing resolution of its steps. Minimum: 1.0 Maximum: 100.0
     */
    public int script_tick_ms;

    /**
     * Stop an OpMode the agent bridge started when no bridge request arrives for this many
     * milliseconds, so a stalled agent cannot leave a mechanism running. 0 disables. Minimum: 0.0
     * Maximum: 600000.0
     */
    public int deadman_ms;

    /**
     * Run the agent bridge's flight recorder for every OpMode run, without an agent: each run's
     * reads land in /sdcard/FIRST/flight/. False: only when an agent starts one.
     */
    public boolean flight_auto;

    /**
     * Rate at which the flight recorder checks the SDK's bulk caches and the Pinpoint for a new
     * read. Keep it above the OpMode's loop rate, or consecutive loops merge into one row. Minimum:
     * 50.0 Maximum: 2000.0
     */
    public double flight_poll_hz;

    /**
     * Milliseconds between flight recorder writes to the file: the most data a robot-app crash can
     * lose, and how fresh a live read of the file is. Minimum: 50.0 Maximum: 10000.0
     */
    public int flight_flush_ms;

    /**
     * Flight recorder capacity in rows; later rows are dropped and the flight marked truncated.
     * Minimum: 1000.0 Maximum: 2000000.0
     */
    public double flight_max_rows;

    /**
     * Number of flights kept on the robot; the oldest are deleted when a new flight starts.
     * Minimum: 1.0 Maximum: 1000.0
     */
    public double flight_keep;
  }

  public static final class CellAAuto {
    /**
     * Pose with the intake at an alliance-side FLOWER's bottom retrieval opening (G418: POLLEN may
     * only be removed from the bottom).
     */
    public Pose flower;

    /**
     * Parking pose: footprint partially inside the alliance LOADING ZONE but clear of the perimeter
     * wall, so the robot earns both LEAVE and PARK at the end of AUTO.
     */
    public Pose park;

    /** Autonomous volley pose for CELL A. */
    public Pose scoreA;

    /** Autonomous volley pose for CELL B. */
    public Pose scoreB;

    /**
     * Bezier control point on the path to score_b; keeps the swept footprint clear of the HIVE
     * frame and on the alliance side.
     */
    public Pose scoreBCp;

    /**
     * Starting pose: touching the perimeter wall, fully on the alliance's side, not in the LOADING
     * ZONE and not touching a FLOWER (G304).
     */
    public Pose start;

    // Shared operational parameters (same for both alliances)
    /**
     * Measured autonomous shoot windows as flat distance_in, shoot_window_ms pairs, written by the
     * Shot Timing Tuner OpMode in exactly this format. Distances must strictly increase;
     * interpolated and clamped at the ends. A volley ends when auto.balls_per_volley_count balls
     * are counted OR this window expires; the window is the jam safety net. auto.shoot_wait_ms is
     * the fallback when this table is empty.
     */
    public double[] shotTimePoints;

    /**
     * Fallback autonomous shoot window (milliseconds), used only when auto.shot_time_points is
     * empty.
     */
    public int shootWaitMs;

    /**
     * SCORING ELEMENTS expected per autonomous volley. A volley ends when this many shots are
     * counted by shooter.ball_detection or the shot window expires.
     */
    public int ballsPerVolleyCount;

    /**
     * Autonomous wait (milliseconds) for the target CELL to be seen upward-facing before giving up
     * on it. The robot never launches at a downward-facing CELL (G417).
     */
    public int tipWaitMs;

    /**
     * Autonomous time (milliseconds) spent at a FLOWER retrieval opening with the intake running.
     */
    public int collectDwellMs;

    /**
     * AUTO park deadline margin (milliseconds): the routine is cancelled and the robot parks once
     * the time left is no more than the estimated drive to the park pose (stopping, then the
     * HIVE-avoiding route) plus this.
     */
    public int parkMarginMs;

    /**
     * Latest AUTO park start (milliseconds before the end): the robot parks at this point even if
     * the estimated drive plus park_margin_ms is shorter, or when no estimate exists.
     */
    public int parkReserveMs;

    /**
     * How often (milliseconds) AUTO re-plans the park route and re-estimates the drive time for the
     * park deadline.
     */
    public int parkUpdateMs;

    /**
     * Transfer motor power (0.0 to 1.0) while the intake is collecting (not feeding the shooter).
     */
    public double transferPower;

    /**
     * CELL A routine: collect POLLEN from the alliance's side-wall FLOWER after the first volley.
     */
    public boolean cellACollectFlower;

    /**
     * CELL A routine: drive around the HIVE on the FLOWER-side lane and launch a second volley into
     * CELL B once it is upward-facing. Without cell_a_collect_flower this fires only POLLEN left
     * over when the TIP stopped the first volley.
     */
    public boolean cellASecondVolley;

    /**
     * CELL B routine: collect POLLEN from the alliance's rear-wall FLOWER and launch a second
     * volley.
     */
    public boolean cellBCollectFlower;
  }

  public static final class CellBAuto {
    /**
     * CELL B routine fallback volley pose for CELL A, used when CELL B never becomes upward-facing.
     */
    public Pose altA;

    /** Waypoint on the CELL B routine's fallback route around the HIVE to alt_a. */
    public Pose altVia;

    /**
     * Pose with the intake at an alliance-side FLOWER's bottom retrieval opening (G418: POLLEN may
     * only be removed from the bottom).
     */
    public Pose flower;

    /**
     * Parking pose: footprint partially inside the alliance LOADING ZONE but clear of the perimeter
     * wall, so the robot earns both LEAVE and PARK at the end of AUTO.
     */
    public Pose park;

    /** Autonomous volley pose for CELL B. */
    public Pose scoreB;

    /**
     * Bezier control point on the path to score_b; keeps the swept footprint clear of the HIVE
     * frame and on the alliance side.
     */
    public Pose scoreBCp;

    /**
     * Starting pose: touching the perimeter wall, fully on the alliance's side, not in the LOADING
     * ZONE and not touching a FLOWER (G304).
     */
    public Pose start;

    // Shared operational parameters (same for both alliances)
    /**
     * Measured autonomous shoot windows as flat distance_in, shoot_window_ms pairs, written by the
     * Shot Timing Tuner OpMode in exactly this format. Distances must strictly increase;
     * interpolated and clamped at the ends. A volley ends when auto.balls_per_volley_count balls
     * are counted OR this window expires; the window is the jam safety net. auto.shoot_wait_ms is
     * the fallback when this table is empty.
     */
    public double[] shotTimePoints;

    /**
     * Fallback autonomous shoot window (milliseconds), used only when auto.shot_time_points is
     * empty.
     */
    public int shootWaitMs;

    /**
     * SCORING ELEMENTS expected per autonomous volley. A volley ends when this many shots are
     * counted by shooter.ball_detection or the shot window expires.
     */
    public int ballsPerVolleyCount;

    /**
     * Autonomous wait (milliseconds) for the target CELL to be seen upward-facing before giving up
     * on it. The robot never launches at a downward-facing CELL (G417).
     */
    public int tipWaitMs;

    /**
     * Autonomous time (milliseconds) spent at a FLOWER retrieval opening with the intake running.
     */
    public int collectDwellMs;

    /**
     * AUTO park deadline margin (milliseconds): the routine is cancelled and the robot parks once
     * the time left is no more than the estimated drive to the park pose (stopping, then the
     * HIVE-avoiding route) plus this.
     */
    public int parkMarginMs;

    /**
     * Latest AUTO park start (milliseconds before the end): the robot parks at this point even if
     * the estimated drive plus park_margin_ms is shorter, or when no estimate exists.
     */
    public int parkReserveMs;

    /**
     * How often (milliseconds) AUTO re-plans the park route and re-estimates the drive time for the
     * park deadline.
     */
    public int parkUpdateMs;

    /**
     * Transfer motor power (0.0 to 1.0) while the intake is collecting (not feeding the shooter).
     */
    public double transferPower;

    /**
     * CELL A routine: collect POLLEN from the alliance's side-wall FLOWER after the first volley.
     */
    public boolean cellACollectFlower;

    /**
     * CELL A routine: drive around the HIVE on the FLOWER-side lane and launch a second volley into
     * CELL B once it is upward-facing. Without cell_a_collect_flower this fires only POLLEN left
     * over when the TIP stopped the first volley.
     */
    public boolean cellASecondVolley;

    /**
     * CELL B routine: collect POLLEN from the alliance's rear-wall FLOWER and launch a second
     * volley.
     */
    public boolean cellBCollectFlower;
  }

  public static final class ParkOnlyAuto {
    /**
     * Parking pose: footprint partially inside the alliance LOADING ZONE but clear of the perimeter
     * wall, so the robot earns both LEAVE and PARK at the end of AUTO.
     */
    public Pose park;

    /**
     * Starting pose: touching the perimeter wall, fully on the alliance's side, not in the LOADING
     * ZONE and not touching a FLOWER (G304).
     */
    public Pose start;

    // Shared operational parameters (same for both alliances)
    /**
     * Measured autonomous shoot windows as flat distance_in, shoot_window_ms pairs, written by the
     * Shot Timing Tuner OpMode in exactly this format. Distances must strictly increase;
     * interpolated and clamped at the ends. A volley ends when auto.balls_per_volley_count balls
     * are counted OR this window expires; the window is the jam safety net. auto.shoot_wait_ms is
     * the fallback when this table is empty.
     */
    public double[] shotTimePoints;

    /**
     * Fallback autonomous shoot window (milliseconds), used only when auto.shot_time_points is
     * empty.
     */
    public int shootWaitMs;

    /**
     * SCORING ELEMENTS expected per autonomous volley. A volley ends when this many shots are
     * counted by shooter.ball_detection or the shot window expires.
     */
    public int ballsPerVolleyCount;

    /**
     * Autonomous wait (milliseconds) for the target CELL to be seen upward-facing before giving up
     * on it. The robot never launches at a downward-facing CELL (G417).
     */
    public int tipWaitMs;

    /**
     * Autonomous time (milliseconds) spent at a FLOWER retrieval opening with the intake running.
     */
    public int collectDwellMs;

    /**
     * AUTO park deadline margin (milliseconds): the routine is cancelled and the robot parks once
     * the time left is no more than the estimated drive to the park pose (stopping, then the
     * HIVE-avoiding route) plus this.
     */
    public int parkMarginMs;

    /**
     * Latest AUTO park start (milliseconds before the end): the robot parks at this point even if
     * the estimated drive plus park_margin_ms is shorter, or when no estimate exists.
     */
    public int parkReserveMs;

    /**
     * How often (milliseconds) AUTO re-plans the park route and re-estimates the drive time for the
     * park deadline.
     */
    public int parkUpdateMs;

    /**
     * Transfer motor power (0.0 to 1.0) while the intake is collecting (not feeding the shooter).
     */
    public double transferPower;

    /**
     * CELL A routine: collect POLLEN from the alliance's side-wall FLOWER after the first volley.
     */
    public boolean cellACollectFlower;

    /**
     * CELL A routine: drive around the HIVE on the FLOWER-side lane and launch a second volley into
     * CELL B once it is upward-facing. Without cell_a_collect_flower this fires only POLLEN left
     * over when the TIP stopped the first volley.
     */
    public boolean cellASecondVolley;

    /**
     * CELL B routine: collect POLLEN from the alliance's rear-wall FLOWER and launch a second
     * volley.
     */
    public boolean cellBCollectFlower;
  }

  public static MatchProfile loadMatchProfile(Alliance alliance) {
    return MatchProfile.loadMatchProfile(alliance);
  }
}
