package org.firstinspires.ftc.teamcode.records;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Immutable parameters for the 2D point-mass ballistics trajectory model with drag and Magnus lift,
 * aimed at the centre of an upward-facing HIVE CELL opening.
 *
 * <p>The exit-velocity fields describe a linear flywheel model: {@code v0} is the measured exit
 * speed produced when the flywheel holds {@code v0ReferenceRpm}, so exit speed and flywheel
 * setpoint convert through {@code v0 / v0ReferenceRpm}. That is what lets the solver pick a
 * flywheel RPM per shot instead of firing everything at one fixed speed.
 */
public record BallisticsParameters(
    double v0, // Exit velocity at v0ReferenceRpm (in/s)
    double k, // Lumped quadratic drag coefficient (1/in)
    double L, // Lumped Magnus lift gain (1/s)
    double launchHeightInches, // Ball release height above the TILES (in)
    double targetHeightInches, // Upward CELL opening centre height above the TILES (in)
    double gInchesPerSec2, // Gravitational acceleration (386.088 in/s²)
    double minHoodAngleDeg, // Minimum hood elevation angle (deg)
    double maxHoodAngleDeg, // Maximum hood elevation angle (deg)
    double minHoodServoPos, // Servo position corresponding to minHoodAngleDeg (0.0 to 1.0)
    double maxHoodServoPos, // Servo position corresponding to maxHoodAngleDeg (0.0 to 1.0)
    double maxMovingSpeedIps, // Maximum allowed robot translation speed for valid shots (in/s)
    double minValidDistanceInches, // Minimum calibrated shot distance (in)
    double maxValidDistanceInches, // Maximum calibrated shot distance (in)
    double leadBiasGain, // Empirical lead calibration gain multiplier
    double v0ReferenceRpm, // Flywheel setpoint at which v0 was measured
    double preferredShotRpm, // Flywheel setpoint the hood aims around, and the solver's floor
    double maxShotRpm, // Highest flywheel setpoint the RPM solver may command
    double v0MarginFraction, // Headroom above the minimum reaching speed (0.15 = 15% faster)
    double flightTimeBaseSec, // Time of flight at zero range: lob rise time, for lead only
    double flightTimeSecPerInch, // Added time of flight per inch of range, for lead only
    double leadMinSpeedIps, // Robot speed below which lead is skipped as estimator noise
    double maxOffAxisDeg // Largest angle off the CELL opening's outward direction to shoot from
    ) {
  public BallisticsParameters {
    if (v0 <= 0) throw new IllegalArgumentException("v0 must be positive");
    if (k < 0) throw new IllegalArgumentException("k must be non-negative");
    if (gInchesPerSec2 <= 0) throw new IllegalArgumentException("g must be positive");
    if (v0ReferenceRpm <= 0) throw new IllegalArgumentException("v0ReferenceRpm must be positive");
    if (preferredShotRpm > maxShotRpm) {
      throw new IllegalArgumentException("preferredShotRpm must not exceed maxShotRpm");
    }
    if (v0MarginFraction < 0) {
      throw new IllegalArgumentException("v0MarginFraction must be non-negative");
    }
  }

  /** Builds the live parameter set from the generated config facade. */
  public static BallisticsParameters fromConfig() {
    var b = config.shooter.ballistics;
    return new BallisticsParameters(
        b.v0,
        b.k,
        b.magnus_l,
        b.launch_height_inches,
        config.field.hive.up_opening_height_in,
        b.g_inches_per_sec2,
        b.min_hood_angle_deg,
        b.max_hood_angle_deg,
        b.min_hood_servo_pos,
        b.max_hood_servo_pos,
        b.max_moving_speed_ips,
        b.min_valid_distance,
        b.max_valid_distance,
        b.lead_bias_gain,
        b.v0_reference_rpm,
        b.preferred_shot_rpm,
        b.max_shot_rpm,
        b.v0_margin_fraction,
        b.flight_time_base_sec,
        b.flight_time_sec_per_inch,
        b.lead_min_speed_ips,
        b.max_off_axis_deg);
  }

  /**
   * Lowest numeric servo position of the hood's travel.
   *
   * <p>The hood linkage may be reversed, i.e. {@code minHoodServoPos > maxHoodServoPos}, since
   * those name the servo positions at the minimum and maximum *angle*, not the numerically smaller
   * and larger positions. Clamp against these accessors rather than against the raw fields, or a
   * reversed linkage produces an inverted range that pins the hood at one end of its travel.
   */
  public double servoTravelLow() {
    return Math.min(minHoodServoPos, maxHoodServoPos);
  }

  /** Highest numeric servo position of the hood's travel. See {@link #servoTravelLow()}. */
  public double servoTravelHigh() {
    return Math.max(minHoodServoPos, maxHoodServoPos);
  }

  /** Estimated time of flight (seconds) for a shot at {@code distanceInches}, for lead only. */
  public double flightTimeAt(double distanceInches) {
    return flightTimeBaseSec + flightTimeSecPerInch * Math.max(0.0, distanceInches);
  }

  /** Exit speed (in/s) produced per unit of flywheel setpoint. */
  public double v0PerRpm() {
    return v0 / v0ReferenceRpm;
  }

  /** Converts a flywheel setpoint to the exit speed the linear flywheel model predicts. */
  public double rpmToV0(double rpm) {
    return rpm * v0PerRpm();
  }

  /** Converts a desired exit speed back to the flywheel setpoint that produces it. */
  public double v0ToRpm(double exitVelocityIps) {
    return exitVelocityIps / v0PerRpm();
  }

  /** Copy with a different exit velocity, for solving or fitting over v0. */
  public BallisticsParameters withV0(double newV0) {
    return new BallisticsParameters(
        newV0,
        k,
        L,
        launchHeightInches,
        targetHeightInches,
        gInchesPerSec2,
        minHoodAngleDeg,
        maxHoodAngleDeg,
        minHoodServoPos,
        maxHoodServoPos,
        maxMovingSpeedIps,
        minValidDistanceInches,
        maxValidDistanceInches,
        leadBiasGain,
        v0ReferenceRpm,
        preferredShotRpm,
        maxShotRpm,
        v0MarginFraction,
        flightTimeBaseSec,
        flightTimeSecPerInch,
        leadMinSpeedIps,
        maxOffAxisDeg);
  }

  /** Copy with different fitted drag and Magnus coefficients, for parameter fitting. */
  public BallisticsParameters withDragAndLift(double newK, double newL) {
    return new BallisticsParameters(
        v0,
        newK,
        newL,
        launchHeightInches,
        targetHeightInches,
        gInchesPerSec2,
        minHoodAngleDeg,
        maxHoodAngleDeg,
        minHoodServoPos,
        maxHoodServoPos,
        maxMovingSpeedIps,
        minValidDistanceInches,
        maxValidDistanceInches,
        leadBiasGain,
        v0ReferenceRpm,
        preferredShotRpm,
        maxShotRpm,
        v0MarginFraction,
        flightTimeBaseSec,
        flightTimeSecPerInch,
        leadMinSpeedIps,
        maxOffAxisDeg);
  }
}
