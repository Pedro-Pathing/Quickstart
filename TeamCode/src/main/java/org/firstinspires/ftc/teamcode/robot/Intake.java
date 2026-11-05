package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;

/**
 * Front intake and middle transfer rollers, plus the G407 four-element inventory.
 *
 * <p>Single writer: commands, {@link ShotController} and OpModes set a target with {@link #run},
 * {@link #feed}, {@link #eject} or {@link #stop}; only {@link #periodic()} writes the motors. That
 * is where the inventory cap is applied, so no caller can drive a fifth SCORING ELEMENT in by
 * accident.
 */
public class Intake {

  /** What the rollers are doing this loop. FEED and EJECT ignore the inventory cap. */
  public enum Mode {
    STOPPED,
    COLLECT,
    FEED,
    EJECT
  }

  private final DcMotorEx intakeFront;
  private final DcMotorEx intakeMid;
  private final @Nullable DigitalChannel entrySensor;
  private final BallInventory inventory;

  private Mode mode = Mode.STOPPED;
  private double targetIntakePower = 0.0;
  private double targetTransferPower = 0.0;
  private boolean blockedByCapacity = false;

  public Intake(HardwareMap hardwareMap) {
    intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
    intakeMid = hardwareMap.get(DcMotorEx.class, "intakeMid");

    intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
    intakeMid.setDirection(DcMotorSimple.Direction.REVERSE);

    var inv = config.inventory;
    DigitalChannel sensor =
        inv.entry_sensor_enabled
            ? hardwareMap.tryGet(DigitalChannel.class, inv.entry_sensor_name)
            : null;
    if (sensor != null) {
      sensor.setMode(DigitalChannel.Mode.INPUT);
    }
    entrySensor = sensor;
    inventory = new BallInventory(inv.capacity_count, sensor != null);
  }

  /** Collect from the floor or a FLOWER: stops itself once the inventory is full. */
  public void run(double intakePower, double transferPower) {
    setTarget(Mode.COLLECT, intakePower, transferPower);
  }

  /** Push SCORING ELEMENTS into the shooter. Not capped: feeding only ever empties the robot. */
  public void feed(double intakePower, double transferPower) {
    setTarget(Mode.FEED, intakePower, transferPower);
  }

  /** Reverse everything to spit out a fifth element (G407) or opponent NECTAR (G408). */
  public void eject(double power) {
    double reverse = -Math.abs(power);
    setTarget(Mode.EJECT, reverse, reverse);
  }

  public void stop() {
    setTarget(Mode.STOPPED, 0.0, 0.0);
  }

  private void setTarget(Mode newMode, double intakePower, double transferPower) {
    mode = newMode;
    targetIntakePower = intakePower;
    targetTransferPower = transferPower;
  }

  public Mode mode() {
    return mode;
  }

  public boolean isRunning() {
    return mode != Mode.STOPPED;
  }

  /** True when the last {@link #periodic()} held the intake off because the robot was full. */
  public boolean isBlockedByCapacity() {
    return blockedByCapacity;
  }

  public BallInventory inventory() {
    return inventory;
  }

  /** G407: the proven predicate {@link BallInventory#mayCollect}; unknown never blocks. */
  private boolean mayCollect() {
    return BallInventory.mayCollect(inventory.isKnown(), inventory.count(), inventory.capacity());
  }

  /** The only place the intake motors are written. Called once per loop by Robot.update(). */
  public void periodic() {
    if (entrySensor != null) {
      boolean raw = entrySensor.getState();
      boolean blocked = config.inventory.entry_sensor_active_low ? !raw : raw;
      inventory.updateEntrySensor(
          blocked,
          System.nanoTime() / 1_000_000L,
          config.inventory.entry_debounce_ms,
          mode == Mode.EJECT);
    } else if ((mode == Mode.COLLECT && mayCollect()) || mode == Mode.EJECT) {
      inventory.markUnknown();
    }

    blockedByCapacity = mode == Mode.COLLECT && !mayCollect();
    boolean powered = mode != Mode.STOPPED && !blockedByCapacity;
    intakeFront.setPower(powered ? targetIntakePower : 0.0);
    intakeMid.setPower(powered ? targetTransferPower : 0.0);
  }

  public Command runIntakeCommand(double intakePower, double transferPower) {
    return Command.build()
        .setStart(() -> run(intakePower, transferPower))
        .setEnd(interrupted -> stop())
        .requiring(this);
  }

  public Command ejectCommand(double power) {
    return Command.build()
        .setStart(() -> eject(power))
        .setEnd(interrupted -> stop())
        .requiring(this);
  }

  public Command stopIntakeCommand() {
    return Command.build().setStart(this::stop).setDone(() -> true).requiring(this);
  }
}
