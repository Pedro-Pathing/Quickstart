package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** Owns the intake, shooter, and continuous-rotation servo hardware. */
public final class GameMechanisms {
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final ServoMotorSubsystem servoMotor;

  public GameMechanisms(HardwareMap hardwareMap) {
    intake = new IntakeSubsystem(hardwareMap);
    shooter = new ShooterSubsystem(hardwareMap);
    servoMotor = new ServoMotorSubsystem(hardwareMap);
    stopAll();
  }

  public void toggleIntakeDirection() {
    intake.toggleDirection();
  }

  public void toggleIntakeEnabled() {
    intake.toggleEnabled();
  }

  /** The shooter intentionally uses hold-to-run behavior. */
  public void setShooterActive(boolean active) {
    shooter.setActive(active);
  }

  public void toggleServoEnabled() {
    servoMotor.toggleEnabled();
  }

  public IntakeSubsystem.Direction getIntakeDirection() {
    return intake.getDirection();
  }

  public boolean isIntakeEnabled() {
    return intake.isEnabled();
  }

  public boolean isServoEnabled() {
    return servoMotor.isEnabled();
  }

  public boolean isShooterActive() {
    return shooter.isActive();
  }

  public void stopAll() {
    intake.stop();
    shooter.stop();
    servoMotor.stop();
  }
}
