package org.firstinspires.ftc.teamcode.localization;

/** Compares live Limelight tag measurements with motion predicted by odometry. */
public final class AprilTagOdometryValidator {
  private final double positionToleranceInches;
  private final double heightToleranceInches;

  private Integer trackedTagId;
  private double tagFieldX;
  private double tagFieldY;
  private double baselineHeight;

  public AprilTagOdometryValidator(
      double positionToleranceInches, double heightToleranceInches) {
    if (positionToleranceInches < 0 || heightToleranceInches < 0) {
      throw new IllegalArgumentException("Validation tolerances cannot be negative");
    }
    this.positionToleranceInches = positionToleranceInches;
    this.heightToleranceInches = heightToleranceInches;
  }

  /** Captures a fixed tag's field position from the current odometry and Limelight measurements. */
  public void captureBaseline(PoseEstimate robotPose, AprilTagObservation tag) {
    double heading = Math.toRadians(robotPose.getHeadingDegrees());
    tagFieldX =
        robotPose.getX()
            + tag.getForwardInches() * Math.cos(heading)
            - tag.getLeftInches() * Math.sin(heading);
    tagFieldY =
        robotPose.getY()
            + tag.getForwardInches() * Math.sin(heading)
            + tag.getLeftInches() * Math.cos(heading);
    baselineHeight = tag.getHeightInches();
    trackedTagId = tag.getId();
  }

  public void clearBaseline() {
    trackedTagId = null;
  }

  public boolean hasBaseline() {
    return trackedTagId != null;
  }

  public int getTrackedTagId() {
    if (trackedTagId == null) {
      throw new IllegalStateException("No AprilTag baseline has been captured");
    }
    return trackedTagId;
  }

  /** Translation inferred from the fixed baseline tag and the current odometry/IMU heading. */
  public PoseEstimate estimateRobotPose(PoseEstimate headingSource, AprilTagObservation tag) {
    if (trackedTagId == null || tag == null || tag.getId() != trackedTagId) {
      return null;
    }
    double heading = Math.toRadians(headingSource.getHeadingDegrees());
    return new PoseEstimate(
        tagFieldX - tag.getForwardInches() * Math.cos(heading)
            + tag.getLeftInches() * Math.sin(heading),
        tagFieldY - tag.getForwardInches() * Math.sin(heading)
            - tag.getLeftInches() * Math.cos(heading),
        headingSource.getHeadingDegrees());
  }

  public ValidationResult validate(PoseEstimate robotPose, AprilTagObservation observedTag) {
    if (trackedTagId == null || observedTag == null || observedTag.getId() != trackedTagId) {
      return ValidationResult.unavailable();
    }

    double heading = Math.toRadians(robotPose.getHeadingDegrees());
    double fieldDeltaX = tagFieldX - robotPose.getX();
    double fieldDeltaY = tagFieldY - robotPose.getY();

    double expectedForward =
        fieldDeltaX * Math.cos(heading) + fieldDeltaY * Math.sin(heading);
    double expectedLeft =
        -fieldDeltaX * Math.sin(heading) + fieldDeltaY * Math.cos(heading);
    double expectedHorizontalDistance = Math.hypot(expectedForward, expectedLeft);

    double forwardError = observedTag.getForwardInches() - expectedForward;
    double leftError = observedTag.getLeftInches() - expectedLeft;
    double positionError = Math.hypot(forwardError, leftError);
    double distanceError =
        observedTag.getHorizontalDistanceInches() - expectedHorizontalDistance;
    double heightError = observedTag.getHeightInches() - baselineHeight;

    boolean passing =
        positionError <= positionToleranceInches
            && Math.abs(heightError) <= heightToleranceInches;
    return new ValidationResult(
        true,
        passing,
        expectedForward,
        expectedLeft,
        expectedHorizontalDistance,
        observedTag.getHorizontalDistanceInches(),
        positionError,
        distanceError,
        heightError);
  }

  public static final class ValidationResult {
    private final boolean available;
    private final boolean passing;
    private final double expectedForwardInches;
    private final double expectedLeftInches;
    private final double expectedDistanceInches;
    private final double measuredDistanceInches;
    private final double positionErrorInches;
    private final double distanceErrorInches;
    private final double heightErrorInches;

    private ValidationResult(
        boolean available,
        boolean passing,
        double expectedForwardInches,
        double expectedLeftInches,
        double expectedDistanceInches,
        double measuredDistanceInches,
        double positionErrorInches,
        double distanceErrorInches,
        double heightErrorInches) {
      this.available = available;
      this.passing = passing;
      this.expectedForwardInches = expectedForwardInches;
      this.expectedLeftInches = expectedLeftInches;
      this.expectedDistanceInches = expectedDistanceInches;
      this.measuredDistanceInches = measuredDistanceInches;
      this.positionErrorInches = positionErrorInches;
      this.distanceErrorInches = distanceErrorInches;
      this.heightErrorInches = heightErrorInches;
    }

    private static ValidationResult unavailable() {
      return new ValidationResult(false, false, 0, 0, 0, 0, 0, 0, 0);
    }

    public boolean isAvailable() {
      return available;
    }

    public boolean isPassing() {
      return passing;
    }

    public double getExpectedForwardInches() {
      return expectedForwardInches;
    }

    public double getExpectedLeftInches() {
      return expectedLeftInches;
    }

    public double getExpectedDistanceInches() {
      return expectedDistanceInches;
    }

    public double getMeasuredDistanceInches() {
      return measuredDistanceInches;
    }

    public double getPositionErrorInches() {
      return positionErrorInches;
    }

    public double getDistanceErrorInches() {
      return distanceErrorInches;
    }

    public double getHeightErrorInches() {
      return heightErrorInches;
    }
  }
}
