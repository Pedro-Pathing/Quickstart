package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Controls the two shooter motors using closed-loop RPM targets. */
public final class ShooterSubsystem {
  private static final double DEGREES_PER_SECOND_PER_RPM = 6.0;

  private final DcMotorEx motor1;
  private final DcMotorEx motor2;
  private volatile boolean active;
  private volatile double targetRpm1 = RobotConfig.Mechanisms.SHOOTER_TARGET_RPM;
  private volatile double targetRpm2 = RobotConfig.Mechanisms.SHOOTER_TARGET_RPM;

  public ShooterSubsystem(HardwareMap hardwareMap) {
    motor1 = hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.SHOOTER_1);
    motor2 = hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.SHOOTER_2);
    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    stop();
  }

  public void setActive(boolean active) {
    this.active = active;
    applyVelocity();
  }

  public void setTargetRpms(double motor1Rpm, double motor2Rpm) {
    targetRpm1 = Math.max(0, motor1Rpm);
    targetRpm2 = Math.max(0, motor2Rpm);
    applyVelocity();
  }

  public boolean isActive() {
    return active;
  }

  public double getTargetRpm1() {
    return targetRpm1;
  }

  public double getTargetRpm2() {
    return targetRpm2;
  }

  public double getActualRpm1() {
    return motor1.getVelocity(AngleUnit.DEGREES) / DEGREES_PER_SECOND_PER_RPM;
  }

  public double getActualRpm2() {
    return motor2.getVelocity(AngleUnit.DEGREES) / DEGREES_PER_SECOND_PER_RPM;
  }

  public void stop() {
    active = false;
    motor1.setVelocity(0, AngleUnit.DEGREES);
    motor2.setVelocity(0, AngleUnit.DEGREES);
  }

  private void applyVelocity() {
    motor1.setVelocity(
        active ? targetRpm1 * DEGREES_PER_SECOND_PER_RPM : 0, AngleUnit.DEGREES);
    motor2.setVelocity(
        active ? targetRpm2 * DEGREES_PER_SECOND_PER_RPM : 0, AngleUnit.DEGREES);
  }
}
