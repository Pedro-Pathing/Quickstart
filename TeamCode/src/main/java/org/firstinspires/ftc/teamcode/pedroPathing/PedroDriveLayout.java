package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.drive.DrivePower;
import org.firstinspires.ftc.teamcode.drive.MecanumKinematics;

/** Converts Pedro wheel order LF/LR/RF/RR through the shared chassis axis conventions. */
public final class PedroDriveLayout {
  private PedroDriveLayout() {}

  public static double[] remap(double[] powers, boolean swapAxes, boolean reverseRotation) {
    double forward = (powers[0] + powers[1] + powers[2] + powers[3]) / 4.0;
    double right = (powers[0] - powers[1] - powers[2] + powers[3]) / 4.0;
    double clockwise = (powers[0] + powers[1] - powers[2] - powers[3]) / 4.0;
    DrivePower result = MecanumKinematics.robotCentric(
        swapAxes ? -right : forward,
        swapAxes ? -forward : right,
        reverseRotation ? -clockwise : clockwise);
    return new double[] {
      result.getFrontLeft(), result.getBackLeft(), result.getFrontRight(), result.getBackRight()
    };
  }
}
