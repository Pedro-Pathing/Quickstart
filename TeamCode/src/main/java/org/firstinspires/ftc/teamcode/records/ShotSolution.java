package org.firstinspires.ftc.teamcode.records;

/** Immutable outcome of the stateless shot solver for one {@link ShotInputs} snapshot. */
public record ShotSolution(
    double targetHoodPosition, // Continuous hood servo position (0.0 to 1.0)
    double targetRpm, // Solved flywheel setpoint (encoder ticks/s)
    double targetExitVelocityIps, // Exit speed the setpoint should produce (in/s)
    double targetAzimuthRad, // Field bearing to the (lead-compensated) opening; NaN if none
    double predictedFlightTimeSec, // Estimated time of flight (seconds)
    double predictedEntryAngleRad, // Reserved for a trajectory model; 0 for table lookups
    double distanceInches, // Horizontal distance used for the lookup (inches)
    boolean isValid, // Within speed, range, approach-side and calibration gates
    String validityReason, // Human-readable reason if invalid
    int targetEpoch, // HiveTracker epoch of the target this was solved for; -1 if none
    BallType ballType // Shot table this solution came from
    ) {

  /** A solution that never authorizes a feed and has no azimuth to aim at (NaN). */
  public static ShotSolution invalid(String reason) {
    return new ShotSolution(
        0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, false, reason, -1, BallType.POLLEN);
  }
}
