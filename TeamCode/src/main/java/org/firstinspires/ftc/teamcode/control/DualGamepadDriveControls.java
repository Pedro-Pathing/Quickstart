package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

/** Maps either gamepad to robot-centric drive commands, with gamepad 1 taking priority. */
public final class DualGamepadDriveControls {
  private static final double INPUT_DEADBAND = 0.05;

  private DualGamepadDriveControls() {}

  public static DriveCommand idle() {
    return new DriveCommand(0, 0, 0, 1);
  }

  public static DriveCommand read(Gamepad gamepad1, Gamepad gamepad2) {
    DriveCommand driver1 =
        fromInputs(
            gamepad1.left_stick_x,
            gamepad1.left_stick_y,
            gamepad1.left_trigger,
            gamepad1.right_trigger,
            1);
    DriveCommand driver2 =
        fromInputs(
            gamepad2.left_stick_x,
            gamepad2.left_stick_y,
            gamepad2.left_trigger,
            gamepad2.right_trigger,
            2);
    return driver1.isActive() || !driver2.isActive() ? driver1 : driver2;
  }

  static DriveCommand fromInputs(
      double leftStickX,
      double leftStickY,
      double leftTrigger,
      double rightTrigger,
      int gamepadNumber) {
    double drive = applyDeadband(-leftStickY);
    double strafe = applyDeadband(leftStickX);
    double rotate = shapeRotation(rightTrigger - leftTrigger);
    return new DriveCommand(drive, strafe, rotate, gamepadNumber);
  }

  /** Gives the analog triggers finer low-speed control while retaining full rotation power. */
  private static double shapeRotation(double value) {
    double deadbanded = applyDeadband(value);
    return Math.copySign(deadbanded * deadbanded, deadbanded);
  }

  private static double applyDeadband(double value) {
    return Math.abs(value) <= INPUT_DEADBAND ? 0 : value;
  }

  public static final class DriveCommand {
    private final double drive;
    private final double strafe;
    private final double rotate;
    private final int gamepadNumber;

    private DriveCommand(double drive, double strafe, double rotate, int gamepadNumber) {
      this.drive = drive;
      this.strafe = strafe;
      this.rotate = rotate;
      this.gamepadNumber = gamepadNumber;
    }

    public double getDrive() {
      return drive;
    }

    public double getStrafe() {
      return strafe;
    }

    public double getRotate() {
      return rotate;
    }

    public int getGamepadNumber() {
      return gamepadNumber;
    }

    public boolean isActive() {
      return drive != 0 || strafe != 0 || rotate != 0;
    }
  }
}
