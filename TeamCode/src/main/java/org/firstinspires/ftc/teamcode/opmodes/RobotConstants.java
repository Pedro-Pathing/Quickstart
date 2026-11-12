package org.firstinspires.ftc.teamcode.opmodes;

/**
 * RobotConstants - Shared constants used across autonomous and teleop programs
 * 
 * This class contains all the common constants used by both BaseAutonomous and BaseMultiPlayerTeleOp
 * to avoid duplication and ensure consistency across the codebase.
 */
public class RobotConstants {
    
    // Position and heading tolerance constants
    public static final double POSITION_TOLERANCE = 3.0;  // 3mm tolerance for position checking (tighter for accuracy)
    public static final double HEADING_TOLERANCE = Math.toRadians(3.0);  // 3 degrees tolerance for heading checking (tighter for accuracy)
    
    // Fine adjustment tolerance constants (for teleop)
    public static final double FINE_POSITION_TOLERANCE = 2.0;  // 2mm tolerance for fine position adjustment (even tighter)
    public static final double FINE_HEADING_TOLERANCE = Math.toRadians(2.0);  // 2 degrees tolerance for fine heading adjustment (even tighter)
    public static final double FINE_ADJUSTMENT_SPEED = 0.3;  // Speed multiplier for fine adjustment movements
    public static final long FINE_ADJUSTMENT_TIMEOUT_MS = 5000;  // 5 second timeout for fine adjustment
    
    // Transition constants (for teleop)
    public static final long TRANSITION_DURATION_MS = 1000;  // 1 second smooth transition period
    
    // Feature toggle flags (for teleop)
    public static final boolean ENABLE_SMOOTH_TRANSITION = true;  // Enable/disable 1-second transition
    public static final boolean ENABLE_CAMERA_FINE_ADJUSTMENT = false;  // Enable/disable camera fine adjustment
    public static final boolean ENABLE_CAMERA_RELOCALIZATION = false;  // Enable/disable continuous camera relocalization

    // Camera relocalization constants (for teleop)
    public static final long CAMERA_RELOCALIZATION_INTERVAL_MS = 500;  // Throttle relocalization updates (2Hz)
    public static final double MAX_RELOCALIZATION_DISTANCE_MM = 600.0;  // Reject relocalization jumps > 600mm
    public static final double MIN_RELOCALIZATION_DISTANCE_MM = 10.0;  // Ignore tiny corrections < 10mm
    public static final long CAMERA_DATA_MAX_STALENESS_MS = 100;  // Reject camera data older than 100ms
    public static final int BLUE_GOAL_APRILTAG_ID = 20;  // Blue alliance goal AprilTag ID

    public static final String RED_GOAL_APRILTAGS_AF = "36,37,38,39";
    public static final String RED_GOAL_APRILTAGS_SF = "32,33,34,35";

    public static final String BLUE_GOAL_APRILTAGS_AF = "40,41,42,43";
    public static final String BLUE_GOAL_APRILTAGS_SF = "44,45,46,47";


    public static final int RED_GOAL_APRILTAG_ID = 24;  // Red alliance goal AprilTag ID

    // Servo constants
    public static final double SERVO_DELAY_MS = 1000;  // Fallback delay if velocity check fails (1 second to ensure motors reach speed)
    public static final long VELOCITY_CHECK_MIN_DELAY_MS = 600;  // Minimum delay before starting velocity check (motors need time to spin up)
    public static final long SERVO_SETTLING_DELAY_MS = 250;  // Time for servo to physically reach target position (250ms)
    public static final double SERVO_RETRACT_DELAY_MS = 700;  // Total time at push position before retract (700ms = 250ms settling + 450ms pushing)
    public static final double SERVO_PUSH_POSITION = 0.4429;  // Servo position to push ball
    public static final double SERVO_REST_POSITION = 0.4;  // Servo rest position
    public static final double LAUNCHER_VELOCITY_TOLERANCE_LOWER = 0.03;  // 3% tolerance below target for launcher speed check
    public static final double LAUNCHER_VELOCITY_TOLERANCE_UPPER = 0.05;  // 5% tolerance above target for launcher speed check
    public static final long GATE_DELAY_MS = 4000;  // Teleop: wait after a launcher reaches speed before its gate feeds (lets it settle)
    
    // Launcher gate servo positions (0..1) - starting values, not yet tuned on the robot
    public static final double POLLEN_SERVO_OPEN_POSITION = 0.65;    // Pollen gate open (feeds launcher)
    public static final double POLLEN_SERVO_CLOSED_POSITION = 0.50;  // Pollen gate closed
    public static final double NECTAR_SERVO_OPEN_POSITION = 0.6;    // Nectar gate open (feeds launcher)
    public static final double NECTAR_SERVO_CLOSED_POSITION = 0.33;  // Nectar gate closed

    // Ball kicker servo positions
    public static final double SLOT1_STARTING_POSITION = 0.44;  // Slot 1 kicker rest position
    public static final double SLOT1_ENDING_POSITION = 0.550;     // Slot 1 kicker extended position
    public static final double SLOT2_STARTING_POSITION = 0.46;  // Slot 2 kicker rest position
    public static final double SLOT2_ENDING_POSITION = 0.6;     // Slot 2 kicker extended position
    public static final double SLOT3_STARTING_POSITION = 0.54;     // Slot 3 kicker rest position
    public static final double SLOT3_ENDING_POSITION = 0.3;     // Slot 3 kicker extended position
    public static final double KICKER_DELAY_MS = 500;           // 500ms delay for ball kickers
    public static final long BALL_TRAVEL_DELAY_MS = 800;        // Time for ball to travel from kicker to launcher area
    
    // Launcher velocity constants (ticks per second)
    public static final int NEAR_LAUNCH_SPEED = 1200;         // Full speed for normal launch (teleop)
    public static final int CENTER_POLLEN_LAUNCH_SPEED = 1125;
    public static final int SIDE_POLLEN_LAUNCH_SPEED = 1125;

    public static final int CENTER_NECTAR_LAUNCH_SPEED = 1250;
    public static final int SIDE_NECTAR_LAUNCH_SPEED = 1250;

    public static final int FAR_LAUNCH_SPEED = 1200;         // High speed for small triangle launch (teleop)
    public static final int LAUNCHER_DUMMY_SPEED = 1000;        // Half speed for dummy launch
    public static final int LAUNCHER_STOP_SPEED = 0;            // Stop speed
    public static final int LAUNCHER_MAX_SPEED = 3000;          // Max speed for variable (trigger) control
    public static final double LAUNCHER_TICKS_PER_REV = 28;     // Encoder ticks per motor revolution
    public static final int LAUNCHER_SPEED_STEP = 5;           // Bumper speed adjustment per press (teleop)

    // Launcher test speeds (ticks per second) - X / A / B buttons in LaunchTestTeleOp
    public static final int LAUNCH_TEST_SPEED_1 = 300;
    public static final int LAUNCH_TEST_SPEED_2 = 2000;
    public static final int LAUNCH_TEST_SPEED_3 = 2500;

    // Intake velocity constants (ticks per second) - starting values, not yet tuned on the robot
    public static final int INTAKE_SPEED = 600;                // Pull game pieces in
    public static final int OUTTAKE_SPEED = -400;              // Push game pieces back out
    public static final int MAX_INTAKE_SPEED = 1200;            // Max speed for variable (trigger) control
    public static final int INTAKE_STOP_SPEED = 0;              // Stop speed
    public static final int INTAKE_SPEED_STEP = 5;             // Bumper speed adjustment per press (teleop)
    public static final int INTAKE_ADJUST_MAX_SPEED = 2000;     // Cap for bumper speed adjustment (teleop)

    // Lift velocity constants (ticks per second) - starting values, not yet tuned on the robot
    public static final int LIFT_UP_SPEED = 800;                // Raise the lift
    public static final int LIFT_DOWN_SPEED = -600;             // Lower the lift
    public static final int MAX_LIFT_SPEED = 100;              // Max speed for variable (joystick) control
    public static final int LIFT_STOP_SPEED = 0;                // Stop speed

    // Driver control constants (for teleop)
    public static final double STICK_DEADZONE = 0.1;            // Ignore joystick values below this
    public static final double TRIGGER_DEADBAND = 0.05;         // Ignore trigger values below this
    public static final double SLOW_MODE_MULTIPLIER = 0.25;     // Drive speed while slow mode is held
    public static final double TEST_MOTOR_POWER = 0.5;          // Power for single-motor wiring tests

    // Goal alignment constants (teleop, both bumpers held) - starting values, not yet tuned on the robot
    public static final double ALIGN_KP = 0.02;                 // Turn power per degree of Limelight tx error
    public static final double ALIGN_MAX_TURN_POWER = 0.4;      // Cap on turn power while aligning
    public static final double ALIGN_MIN_TURN_POWER = 0.05;     // Minimum turn power to overcome friction outside tolerance
    public static final double ALIGN_TOLERANCE_DEG = 1.5;       // tx within this is considered aligned

    // Test autonomous constants (LinearAuto, CurveAuto)
    public static final double TEST_PATH_SPEED = 0.5;           // Max path speed for simple test paths

    // Autonomous-specific constants
    public static final boolean ENABLE_BALL_COLLECTION_SEQUENCE = true;  // Enable/disable ball collection after initial launch
    public static final double BALL_COLLECTION_SPEED = 0.25;  // 1/4 speed for ball collection
    public static final double INTAKE_MOVEMENT_SPEED = 0.3;  // 1/2 speed for slow intake movement
    public static final long LAUNCH_DELAY_MS = 2000;  // 2 second delay between launches
    public static final long INTAKE_DURATION_MS = 3000;  // 3 seconds for ball collection
    public static final long APRIL_TAG_TIMEOUT_MS = 5000;  // 5 second timeout for AprilTag detection
    public static final long PATH_TIMEOUT_MS = 4000;  // 4 second timeout for path following (increased for heading settling)
    public static final long TURN_TIMEOUT_MS = 1500;  // 1.5 second timeout for a turn in place
    public static final long PARK_PATH_TIMEOUT_MS = 7000;  // 7 second timeout for the RETURN_HOME park path (~100" curve around the hive)
    public static final double PARK_WALL_CLEARANCE_IN = 2.0;  // How far the RETURN_HOME park curve bows away from the side wall (inches) so the robot can turn without hitting it
    public static final long INTAKE_PATH_TIMEOUT_MS = 5000;  // 5 second timeout for slow intake movement
    public static final long HEADING_SETTLING_DELAY_MS = 200;  // 200ms delay after path completes to allow heading to settle before launch
    public static final double SHORT_PATH_THRESHOLD_MM = 150.0;  // Paths shorter than this use constant heading, longer paths use heading interpolation
    public static final double AUTON_SPEED_MULTIPLIER = 0.8;
    public static final long AUTONOMOUS_RETURN_HOME_DEADLINE_MS = 28000;  // 28 seconds - Force return home to ensure completion within 30 seconds

    public static final int LIFT_TICKS_ATON = 500;
    // TeleOp drive-to-shooting-position (both gamepad1 bumpers held inside the zone)
    public static final double SHOOTING_ZONE_MIN_X = 12;        // Zone the robot must be in to start (inches)
    public static final double SHOOTING_ZONE_MAX_X = 70;
    public static final double SHOOTING_ZONE_MIN_Y = 96;

    public static final double RED_SF_SHOOTING_ZONE_MIN_X = 12;        // Zone the robot must be in to start (inches)
    public static final double RED_SF_SHOOTING_ZONE_MAX_X = 70;
    public static final double RED_SF_SHOOTING_ZONE_MAX_Y = 48;

    public static final double RED_SF_SHOOTING_POSITION_HEADING_DEG = 270;

    public static final double RED_AF_SHOOTING_POSITION_X = 57;        // Where it drives to (inches)
    public static final double RED_AF_SHOOTING_POSITION_Y = 10;
    public static final double RED_AF_SHOOTING_POSITION_HEADING_DEG = 90;

    public static final double RED_SF_SHOOTING_POSITION_X = 61;        // Where it drives to (inches)
    public static final double RED_SF_SHOOTING_POSITION_Y = 134;

    public static final double BLUE_SF_SHOOTING_ZONE_MIN_X = 87;        // Zone the robot must be in to start (inches)
    public static final double BLUE_SF_SHOOTING_ZONE_MAX_X = 135;
    public static final double BLUE_SF_SHOOTING_ZONE_MAX_Y = 48;

    public static final double BLUE_AF_SHOOTING_POSITION_X = 84;        // Where it drives to (inches)
    public static final double BLUE_AF_SHOOTING_POSITION_Y = 134;
    public static final double BLUE_AF_SHOOTING_POSITION_HEADING_DEG = 270;

    public static final double BLUE_SF_SHOOTING_POSITION_X = 83;        // Where it drives to (inches)
    public static final double BLUE_SF_SHOOTING_POSITION_Y = 14;

    public static final double BLUE_SF_SHOOTING_POSITION_HEADING_DEG = 90;

    public static final double SHOOTING_POSITION_PATH_SPEED = 0.5;  // Max path speed while driving there
    public static final long LIFT_TIMEOUT_MS = 2000;  // 2 second timeout for an autonomous LIFT_UP / LIFT_DOWN move
}
