package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Uses Pedro's vector allocation and motor caching with the installed chassis correction. */
public final class RobotMecanum extends Mecanum {
  private double previousXVelocity = Double.NaN;
  private double previousYVelocity = Double.NaN;

  public RobotMecanum(HardwareMap hardwareMap, MecanumConstants constants) {
    super(hardwareMap, constants);
    updateConstants();
  }

  @Override
  public void updateConstants() {
    super.updateConstants();
    // Pedro 2.1.2's velocity setters do not rebuild its wheel vectors. Refresh after tuning.
    if (constants.xVelocity != previousXVelocity || constants.yVelocity != previousYVelocity) {
      double theta = Math.atan2(-constants.yVelocity, constants.xVelocity);
      constants.frontLeftVector = new Vector(1, theta);
      vectors = new Vector[] {
        new Vector(1, theta), new Vector(1, -theta),
        new Vector(1, -theta), new Vector(1, theta)
      };
      previousXVelocity = constants.xVelocity;
      previousYVelocity = constants.yVelocity;
    }
  }

  @Override
  public void runDrive(double[] powers) {
    super.runDrive(PedroDriveLayout.remap(
        powers, RobotConfig.Drive.SWAP_FORWARD_AND_STRAFE, RobotConfig.Drive.REVERSE_ROTATION));
  }
}
