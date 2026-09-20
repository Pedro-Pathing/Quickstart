package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Controls only the intake motor. */
public final class IntakeSubsystem {
  public enum Direction {
    FORWARD(1.0),
    REVERSE(-1.0);

    private final double multiplier;

    Direction(double multiplier) {
      this.multiplier = multiplier;
    }
  }

  private final DcMotor motor;
  private volatile Direction direction = Direction.FORWARD;
  private volatile boolean enabled;

  public IntakeSubsystem(HardwareMap hardwareMap) {
    motor = hardwareMap.get(DcMotor.class, RobotConfig.Hardware.INTAKE);
    stop();
  }

  public void toggleDirection() {
    direction = direction == Direction.FORWARD ? Direction.REVERSE : Direction.FORWARD;
    applyPower();
  }

  public void toggleEnabled() {
    enabled = !enabled;
    applyPower();
  }

  public Direction getDirection() {
    return direction;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void stop() {
    enabled = false;
    motor.setPower(0);
  }

  private void applyPower() {
    motor.setPower(
        enabled ? RobotConfig.Mechanisms.INTAKE_POWER * direction.multiplier : 0);
  }
}
