package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Controls only the continuous-rotation servo motor. */
public final class ServoMotorSubsystem {
  private final CRServo servo;
  private volatile boolean enabled;

  public ServoMotorSubsystem(HardwareMap hardwareMap) {
    servo = hardwareMap.get(CRServo.class, RobotConfig.Hardware.SERVO_MOTOR);
    stop();
  }

  public void toggleEnabled() {
    enabled = !enabled;
    servo.setPower(enabled ? RobotConfig.Mechanisms.SERVO_POWER : 0);
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void stop() {
    enabled = false;
    servo.setPower(0);
  }
}
