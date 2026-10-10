package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localization.AprilTagOdometryValidator.ValidationResult;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroAprilTagValidation;

public final class PedroValidationTelemetry {
  private PedroValidationTelemetry() {}

  public static void add(Telemetry telemetry, PedroAprilTagValidation validation) {
    telemetry.addData("Pedro / LL Check", validation.getStatus());
    telemetry.addData("Baseline Tag", validation.getTrackedTagId() == null
        ? "None" : validation.getTrackedTagId());
    ValidationResult result = validation.getResult();
    if (result != null && result.isAvailable()) {
      PoseEstimate vision = validation.getVisionPose();
      telemetry.addData("LL-inferred Position", "X %.2f, Y %.2f in", vision.getX(), vision.getY());
      telemetry.addData("Horizontal Distance", "Pedro expected %.2f, LL %.2f in",
          result.getExpectedDistanceInches(), result.getMeasuredDistanceInches());
      telemetry.addData("Position Error", "%.2f in", result.getPositionErrorInches());
      telemetry.addData("Distance Error", "%.2f in", result.getDistanceErrorInches());
      telemetry.addData("Height Drift", "%.2f in", result.getHeightErrorInches());
    }
  }
}
