package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/** Central configuration for hardware names and robot-wide tuning values. */
public final class RobotConfig {
  private RobotConfig() {}

  public static final class Hardware {
    public static final String FRONT_LEFT = "frontLeftMotor";
    public static final String FRONT_RIGHT = "frontRightMotor";
    public static final String BACK_LEFT = "backLeftMotor";
    public static final String BACK_RIGHT = "backRightMotor";
    public static final String PINPOINT = "pinpoint";
    // Physical bus selection is made in the active Robot Controller hardware configuration.
    public static final int PINPOINT_I2C_BUS = 1;
    public static final String LIMELIGHT = "limelight";
    public static final String IMU = "imu";
    public static final String INTAKE = "intake";
    public static final String SHOOTER_1 = "Shoot1";
    public static final String SHOOTER_2 = "Shoot2";
    public static final String SERVO_MOTOR = "servoMotor";

    private Hardware() {}
  }

  public static final class Drive {
    /**
     * The installed drivetrain maps the standard mecanum forward/strafe patterns to the opposite
     * physical axes and directions. Keep this enabled for the current wiring.
     */
    public static final boolean SWAP_FORWARD_AND_STRAFE = true;
    public static final boolean REVERSE_ROTATION = true;

    private Drive() {}
  }

  public static final class Localization {
    public static RevHubOrientationOnRobot imuOrientation() {
      return new RevHubOrientationOnRobot(
          RevHubOrientationOnRobot.LogoFacingDirection.UP,
          RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    public static final int APRILTAG_PIPELINE = 0;
    public static final int POLL_RATE_HZ = 100;
    public static final long APRILTAG_RESULT_STALE_MS = 250;

    public static final double VALIDATION_POSITION_TOLERANCE_INCHES = 3.0;
    public static final double VALIDATION_HEIGHT_TOLERANCE_INCHES = 2.0;

    private Localization() {}
  }

  public static final class Pinpoint {
    // Preserves the existing pod specification; confirm against the installed pods.
    public static final double TICKS_PER_INCH = 1000.0 / (2.0 * Math.PI * 0.6889);
    // Forward/X pod: positive offset left of center. Strafe/Y pod: positive offset ahead of center.
    public static final double FORWARD_POD_Y_INCHES = 0.0;
    public static final double STRAFE_POD_X_INCHES = 0.0;
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_DIRECTION =
        GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection STRAFE_DIRECTION =
        GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private Pinpoint() {}
  }

  public static final class Mechanisms {
    public static final double INTAKE_POWER = 1.0;
    public static final double SHOOTER_TARGET_RPM = 6000.0;
    public static final double SERVO_POWER = 1.0;

    private Mechanisms() {}
  }

  public static final class Diagnostics {
    public static final double DRIVE_MOTOR_TEST_MAX_POWER = 0.35;
    public static final double SHOOTER_INITIAL_RPM = 6000.0;
    public static final double SHOOTER_RPM_STEP = 100.0;

    private Diagnostics() {}
  }

  public static final class Autonomous {
    public static final double POSITION_TOLERANCE_INCHES = 1.5;
    public static final double HEADING_TOLERANCE_DEGREES = 2.0;
    public static final double MAX_DRIVE_POWER = 0.6;
    public static final double MAX_TURN_POWER = 0.5;
    public static final double WAYPOINT_TIMEOUT_SECONDS = 5.0;
    public static final double SETTLE_TIME_SECONDS = 0.3;

    public static final double TRANSLATION_KP = 0.05;
    public static final double TRANSLATION_KI = 0.002;
    public static final double TRANSLATION_KD = 0.01;
    public static final double TRANSLATION_MAX_INTEGRAL = 0.5;
    public static final double TRANSLATION_DEADBAND = 0.5;

    public static final double HEADING_KP = 0.03;
    public static final double HEADING_KI = 0.001;
    public static final double HEADING_KD = 0.005;
    public static final double HEADING_MAX_INTEGRAL = 0.3;
    public static final double HEADING_DEADBAND = 1.0;

    private Autonomous() {}
  }
}
