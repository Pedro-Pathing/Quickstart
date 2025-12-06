package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * Hardware configuration class for the robot
 * Contains all hardware mappings and initialization
 */
public class HardwareConfigAuto {


    // Intake system
    public DcMotor intakeMotor;
    public DcMotor transferMotor;

    // Shooter motor
    public DcMotorEx shooterMotor; // Single shooter with encoder

    // Servos
    public Servo blockerServo;
    //public Servo Lclimb;
    //public Servo Rclimb;
    
    public Servo indexingServo;
    public Servo climberServo;
    public Servo turretServo;

    // Limelight
    public Limelight3A limelight;

    // GoBUILDA Pinpoint Odometry Computer (for X/Y position only)
    public GoBildaPinpointDriver pinpoint;
    
    // Control Hub IMU (for heading - more stable than Pinpoint IMU)
    public IMU imu;

    // RGB Indicator Light (PWM controlled)
    public ServoImplEx lightServo;
    
    // Servo positions
    public static final double BLOCKER_BLOCKED = 0.35;
    public static final double BLOCKER_UNBLOCKED = 0.65;

    // Indexer servo positions
    public static final double INDEXER_INDEXED = 0.45;   // Indexed position (ball ready)
    public static final double INDEXER_MIDDLE = 0.925;   // Middle/default position


    // Light servo positions (PWM-controlled LED colors)
    public static final double LIGHT_OFF = 0.0;          // LED off
    public static final double LIGHT_RED = 0.277;        // Red (unused - no mid sensor)
    public static final double LIGHT_ORANGE = 0.333;     // Orange - 1 ball (minimum)
    public static final double LIGHT_YELLOW = 0.388;     // Yellow - 2 balls
    public static final double LIGHT_GREEN = 0.5;        // Green - 3 balls (full)
    public static final double LIGHT_BLUE = 0.666;       // Blue - shooter ready
    public static final double LIGHT_PURPLE = 0.722;     // Purple - actively shooting
    public static final double LIGHT_WHITE = 1.0;        // White
    
    // Climber servo positions
    public static final double CLIMBER_DOWN = 0.15;      // Unclimbed/starting position
    public static final double CLIMBER_UP = 0.925;       // Climbed position
    
    // Turret servo constants (calibrated positions)
    public static final double TURRET_CENTER_POSITION = 0.51;       // 0 degrees (center)
    public static final double TURRET_90_POSITION = 0.7633;         // 90 degrees (right)
    public static final double TURRET_NEG90_POSITION = 0.2561;      // -90 degrees (left)
    
    // Turret angle limits (physical range)
    public static final double TURRET_MIN_ANGLE = -90.0;            // Minimum turret angle (left limit)
    public static final double TURRET_MAX_ANGLE = 180.0;             // Maximum turret angle (right limit)
    
    // Turret calculation constants (derived from calibration)
    public static final double TURRET_DEGREES_PER_SERVO_UNIT = 355.0;  // Degrees of turret rotation per full servo travel
    public static final double TURRET_HOME_POSITION = 0.51;            // Center position (0 degrees turret angle)
    
    // Goal positions relative to robot starting position (in mm)
    // Robot starts at (0, 0) - goals are 6ft (1828.8mm) to the side
    public static final double GOAL_X = 0.0;                        // Goals are directly to the side, not forward/back
    public static final double GOAL_Y_RED = 1828.8;                 // Red goal: 6ft to the LEFT (positive Y)
    public static final double GOAL_Y_BLUE = -1828.8;               // Blue goal: 6ft to the RIGHT (negative Y)

    // Shooter settings (ticks per second)
    public static final double SHOOTER_DEFAULT_TPS = 1350.0;  // Default when no Limelight target
    public static final double SHOOTER_MIN_TPS = 1250.0;      // Minimum shooter speed
    public static final double SHOOTER_MAX_TPS = 2000.0;      // Maximum shooter speed (2000 RPM)
    public static final double SHOOTER_READY_THRESHOLD = 100.0; // TPS tolerance to consider "ready"
    
    // Limelight area to shooter TPS mapping (calibrated values from testing)
    // Calibration points from field testing (sorted by area, high to low):
    // - 4.3% area = 1350 TPS (closest)
    // - 1.5% area = 1400 TPS
    // - 0.88% area = 1500 TPS
    // - 0.7% area = 1650 TPS
    // - 0.4% area = 1750 TPS
    // - 0.3% area = 1950 TPS (farthest)
    public static final double LIMELIGHT_AREA_1 = 4.3;    // Closest
    public static final double LIMELIGHT_AREA_2 = 1.5;    
    public static final double LIMELIGHT_AREA_3 = 0.88;   
    public static final double LIMELIGHT_AREA_4 = 0.7;    
    public static final double LIMELIGHT_AREA_5 = 0.4;    
    public static final double LIMELIGHT_AREA_6 = 0.3;    // Farthest
    public static final double SHOOTER_TPS_1 = 1350.0;    // TPS at 4.3%
    public static final double SHOOTER_TPS_2 = 1400.0;    // TPS at 1.5%
    public static final double SHOOTER_TPS_3 = 1500.0;    // TPS at 0.88%
    public static final double SHOOTER_TPS_4 = 1600.0;    // TPS at 0.7%
    public static final double SHOOTER_TPS_5 = 1700.0;    // TPS at 0.4%
    public static final double SHOOTER_TPS_6 = 1850.0;    // TPS at 0.3%
    
    // Motor powers
    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSFER_POWER = 1.1;
    public static final double TRANSFER_POWER_FAR = 0.75;  // Reduced transfer speed when far away (area < 0.5)
    public static final double SHOOTER_MAX_POWER = 1.0;

    // Limelight auto-align settings
    public static final double LIMELIGHT_KP = 0.02;  // Proportional gain for alignment
    public static final double LIMELIGHT_MIN_POWER = 0.1;  // Minimum turn power
    public static final double LIMELIGHT_MAX_POWER = 0.5;  // Maximum turn power
    public static final double LIMELIGHT_TOLERANCE = 2.0;  // Degrees tolerance for alignment
    public static final double SHOOTER_READY_ALIGNMENT_TOLERANCE_CLOSE = 5.0;  // Degrees tolerance when close (area >= 0.5)
    public static final double SHOOTER_READY_ALIGNMENT_TOLERANCE_FAR = 2.0;    // Degrees tolerance when far (area < 0.5)
    public static final double APRILTAG_AREA_CLOSE_THRESHOLD = 0.5;  // Area threshold for close vs far distance
    
    // AprilTag auto-aim settings (3D position-based aiming)
    // The goal is located behind each AprilTag (in the direction the tag faces)
    // Works for both tag 20 (left) and tag 24 (right) - same calculation for both
    // System calculates target TX angle based on 3D position of tag and goal
    public static final double APRILTAG_GOAL_OFFSET_MM = 508.0;  // 20 inches behind tag in millimeters
    
    // AprilTag field positions (mm) - used for position reset from camera-relative pose
    // Tag 20 is near the blue goal (5ft 18in = 78in = 1981.2mm to the right, negative Y)
    // Tag 24 is near the red goal (5ft 18in = 78in = 1981.2mm to the left, positive Y)
    public static final double APRILTAG_20_X = 0.0;      // Tag 20 X position on field
    public static final double APRILTAG_20_Y = -1981.2;  // Tag 20 Y position (78in right)
    public static final double APRILTAG_24_X = 0.0;      // Tag 24 X position on field
    public static final double APRILTAG_24_Y = 1981.2;   // Tag 24 Y position (78in left)
    
    // AprilTag facing directions (degrees) - direction the tag faces on the field
    // When robot faces same direction as tag, IMU reads these values
    public static final double APRILTAG_20_FACING = 50.0;   // Tag 20 faces 50° (angled toward center)
    public static final double APRILTAG_24_FACING = -50.0;  // Tag 24 faces -50° (angled toward center)

    // Autonomous drive constants
    public static final double COUNTS_PER_MOTOR_REV = 537.7;    // Encoder counts per revolution (for RUN_TO_POSITION mode)
    public static final double WHEEL_DIAMETER_MM = 96.0;        // Wheel diameter in mm
    public static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_MM);
    public static final double DRIVE_HEADING_TOLERANCE = 2.0;    // Heading tolerance in degrees for rotation

    // Hardware map reference
    private HardwareMap hwMap;

    /**
     * Initialize all hardware devices
     * @param ahwMap The hardware map from the OpMode
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Initialize intake system
        intakeMotor = hwMap.get(DcMotor.class, "intake_motor");
        transferMotor = hwMap.get(DcMotor.class, "transfer_motor");
        
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize shooter motor
        shooterMotor = hwMap.get(DcMotorEx.class, "shooter_motor");
        
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set shooter motor to use encoder for velocity control
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        blockerServo = hwMap.get(Servo.class, "blocker_servo");
        //Lclimb = hwMap.get(Servo.class, "Lclimb");
        //Rclimb = hwMap.get(Servo.class, "Rclimb");
        
        indexingServo = hwMap.get(Servo.class, "indexing_servo");
        climberServo = hwMap.get(Servo.class, "climber_servo");
        turretServo = hwMap.get(Servo.class, "turret_servo");
        
        
        // Set initial servo positions
        blockerServo.setPosition(BLOCKER_BLOCKED);
        indexingServo.setPosition(0.94);  // Start at middle position
        climberServo.setPosition(CLIMBER_DOWN);  // Start at down position
        turretServo.setPosition(TURRET_HOME_POSITION);  // Start at center position

        // Initialize Limelight 3A
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Switch to pipeline 0
        limelight.start();  // Start polling

        // Initialize GoBUILDA Pinpoint Odometry Computer (with built-in IMU)
        //pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        
        // Configure Pinpoint offsets (distance from center of robot to Pinpoint in mm)
        // X = forward/backward offset, Y = left/right (strafe) offset
        //pinpoint.setOffsets(55.0, 12.5, org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM);
        
        // Set encoder resolution for 35mm diameter SWYFT odometry pods
        //pinpoint.setEncoderResolution(35, org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM);
        
        // Set encoder directions (X = forward, Y = reversed)
        //pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                       //GoBildaPinpointDriver.EncoderDirection.REVERSED);
        
        // Reset position to 0,0,0 (we'll use Control Hub IMU for heading instead)
        //pinpoint.resetPosAndIMU();
        
        // Initialize Control Hub IMU (more stable heading than Pinpoint IMU)
        imu = hwMap.get(IMU.class, "imu");
        
        // Set hub orientation - adjust if your hub is mounted differently
        // Logo facing UP, USB facing FORWARD is a common orientation
        IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(imuParameters);
        imu.resetYaw();  // Reset heading to 0

        // Initialize RGB Indicator Light
        lightServo = hwMap.get(ServoImplEx.class, "light_servo");
        lightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));  // Set PWM range

        unload();
    }


    public void store() {
        indexingServo.setPosition(INDEXER_INDEXED);
    }

    public void unload() {
        indexingServo.setPosition(INDEXER_MIDDLE);
    }
    /**
     * Stop all motors
     */
    public void stopAllMotors() {
        
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        
        shooterMotor.setPower(0);
    }
}

