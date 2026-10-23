package org.firstinspires.ftc.teamcode.control;

/**
 * Decides when TeleOp drives from the sticks and when it holds a fixed field pose.
 *
 * <p>Driver input always wins. Once the sticks and triggers are released the robot brakes until it
 * has nearly stopped (or a timeout passes), then the pose at that moment becomes the hold target.
 * Anything that pushes the robot afterwards is corrected back toward that target until the driver
 * moves again. Speeds are measured from successive poses, so the class is hardware independent.
 */
public final class PositionHoldControls {
  public enum Mode {
    /** The driver is commanding the drivetrain. */
    DRIVER,
    /** The driver let go; braking while the robot coasts to a stop. */
    SETTLING,
    /** Actively returning to, and staying at, the captured pose. */
    HOLDING
  }

  /** What the OpMode must do to the follower on this loop. */
  public enum Transition {
    NONE,
    /** Leave hold and return control to the sticks. */
    START_DRIVER,
    /** Begin holding the pose given by {@link #getHoldX()}, {@link #getHoldY()}, heading. */
    START_HOLD
  }

  private final double settledSpeedInchesPerSecond;
  private final double settledTurnRadiansPerSecond;
  private final double settleTimeoutSeconds;

  private Mode mode = Mode.DRIVER;
  private boolean hasLastPose;
  private double lastX;
  private double lastY;
  private double lastHeading;
  private double lastTime;
  private double speed;
  private double turnSpeed;
  private double settleStart;
  private double holdX;
  private double holdY;
  private double holdHeading;

  public PositionHoldControls(
      double settledSpeedInchesPerSecond,
      double settledTurnRadiansPerSecond,
      double settleTimeoutSeconds) {
    this.settledSpeedInchesPerSecond = settledSpeedInchesPerSecond;
    this.settledTurnRadiansPerSecond = settledTurnRadiansPerSecond;
    this.settleTimeoutSeconds = settleTimeoutSeconds;
  }

  /**
   * Advances the state machine by one loop.
   *
   * @param driverActive whether either gamepad is commanding movement
   * @param x current field X in inches
   * @param y current field Y in inches
   * @param headingRadians current heading in radians
   * @param nowSeconds a monotonic timestamp
   */
  public Transition update(
      boolean driverActive, double x, double y, double headingRadians, double nowSeconds) {
    updateSpeeds(x, y, headingRadians, nowSeconds);

    if (driverActive) {
      Mode previous = mode;
      mode = Mode.DRIVER;
      return previous == Mode.HOLDING ? Transition.START_DRIVER : Transition.NONE;
    }

    switch (mode) {
      case DRIVER:
        mode = Mode.SETTLING;
        settleStart = nowSeconds;
        // Fall through so an already-stationary robot locks on the same loop.
      case SETTLING:
        boolean stopped =
            speed <= settledSpeedInchesPerSecond && turnSpeed <= settledTurnRadiansPerSecond;
        if (stopped || nowSeconds - settleStart >= settleTimeoutSeconds) {
          holdX = x;
          holdY = y;
          holdHeading = headingRadians;
          mode = Mode.HOLDING;
          return Transition.START_HOLD;
        }
        return Transition.NONE;
      case HOLDING:
      default:
        return Transition.NONE;
    }
  }

  private void updateSpeeds(double x, double y, double heading, double now) {
    double elapsed = now - lastTime;
    if (hasLastPose && elapsed > 0) {
      speed = Math.hypot(x - lastX, y - lastY) / elapsed;
      turnSpeed = Math.abs(normalizeRadians(heading - lastHeading)) / elapsed;
    } else if (!hasLastPose) {
      // Unknown motion on the first sample: wait for a real measurement before locking.
      speed = Double.POSITIVE_INFINITY;
      turnSpeed = Double.POSITIVE_INFINITY;
    }
    hasLastPose = true;
    lastX = x;
    lastY = y;
    lastHeading = heading;
    lastTime = now;
  }

  /** Planar distance from the hold target, or 0 when not holding. */
  public double distanceFromHold(double x, double y) {
    return mode == Mode.HOLDING ? Math.hypot(holdX - x, holdY - y) : 0;
  }

  /** Signed heading error to the hold target in radians, or 0 when not holding. */
  public double headingErrorFromHold(double headingRadians) {
    return mode == Mode.HOLDING ? normalizeRadians(holdHeading - headingRadians) : 0;
  }

  public Mode getMode() {
    return mode;
  }

  public double getHoldX() {
    return holdX;
  }

  public double getHoldY() {
    return holdY;
  }

  public double getHoldHeading() {
    return holdHeading;
  }

  public double getSpeed() {
    return speed;
  }

  private static double normalizeRadians(double radians) {
    return Math.atan2(Math.sin(radians), Math.cos(radians));
  }
}
