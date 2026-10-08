package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Applies edge-triggered, per-motor RPM adjustments from either gamepad. */
public final class DualShooterRpmControls {
  private double motor1Rpm = RobotConfig.Diagnostics.SHOOTER_INITIAL_RPM;
  private double motor2Rpm = RobotConfig.Diagnostics.SHOOTER_INITIAL_RPM;
  private boolean previousA;
  private boolean previousB;
  private boolean previousX;
  private boolean previousY;

  public void update(Gamepad gamepad1, Gamepad gamepad2) {
    updateButtons(
        gamepad1.a || gamepad2.a,
        gamepad1.b || gamepad2.b,
        gamepad1.x || gamepad2.x,
        gamepad1.y || gamepad2.y);
  }

  void updateButtons(boolean aDown, boolean bDown, boolean xDown, boolean yDown) {
    if (xDown && !previousX) {
      motor1Rpm = Math.min(6000, motor1Rpm + RobotConfig.Diagnostics.SHOOTER_RPM_STEP);
    }
    if (aDown && !previousA) {
      motor1Rpm = Math.max(0, motor1Rpm - RobotConfig.Diagnostics.SHOOTER_RPM_STEP);
    }
    if (yDown && !previousY) {
      motor2Rpm = Math.min(6000, motor2Rpm + RobotConfig.Diagnostics.SHOOTER_RPM_STEP);
    }
    if (bDown && !previousB) {
      motor2Rpm = Math.max(0, motor2Rpm - RobotConfig.Diagnostics.SHOOTER_RPM_STEP);
    }

    previousA = aDown;
    previousB = bDown;
    previousX = xDown;
    previousY = yDown;
  }

  public double getMotor1Rpm() {
    return motor1Rpm;
  }

  public double getMotor2Rpm() {
    return motor2Rpm;
  }
}
