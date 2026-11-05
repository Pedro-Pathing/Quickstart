package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Spins one or both flywheel motors at {@link #targetVelocity} and reports each motor's velocity,
 * peak velocity, and time to reach 95% of target.
 *
 * <p>Runs the hub's velocity loop with the SDK default gains, not {@code shooter.motor_pidf}.
 *
 * <p>Controls: X stops the motors (latched), A restarts the run, Y resets the peak/timing stats.
 */
@TeleOp(name = "Shooter Max Velocity Test", group = "Test")
@Configurable
public class MaxVelocityTest extends LinearOpMode {
  public enum EnabledMotors {
    SHOOTER1,
    SHOOTER2,
    BOTH
  }

  public static String name = "shooter";
  public static String name2 = "shooter2";

  public static EnabledMotors enabled = EnabledMotors.BOTH;

  // Both motors drive the same flywheel from opposite sides, so they must spin in opposite
  // directions — these defaults match Shooter (shooter FORWARD, shooter2 REVERSE).
  public static boolean shooter1forward = true;
  public static boolean shooter2forward = false;

  public static Double power = 1.0;
  public static Double targetVelocity = 1000.0;

  // SDK default velocity gains (the ExpansionHubMotorControllerVelocityParams on REV's motor
  // types). Written explicitly because Shooter's shooter.motor_pidf stays on the hub across an app
  // restart or deploy until the hub is power-cycled, and would otherwise be what this test runs
  // with.
  private static final PIDFCoefficients DEFAULT_VELOCITY_PIDF =
      new PIDFCoefficients(10, 3, 0, 0, MotorControlAlgorithm.LegacyPID);

  @Override
  public void runOpMode() throws InterruptedException {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    org.firstinspires.ftc.teamcode.utilities.OpModeUtil.reportConfig(telemetry);
    ElapsedTime runtime = new ElapsedTime();
    boolean running = true;
    boolean time_recorded = false;
    boolean time2_recorded = false;
    String timeTaken = "0";
    String timeTaken2 = "0";
    double maxMotorVelocity = 0;
    double maxMotor2Velocity = 0;

    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    DcMotorEx testMotor = hardwareMap.get(DcMotorEx.class, name);
    DcMotorEx testMotor2 = hardwareMap.get(DcMotorEx.class, name2);
    // FLOAT so a disabled motor coasts with the flywheel instead of braking against the other one.
    testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    testMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DEFAULT_VELOCITY_PIDF);
    testMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DEFAULT_VELOCITY_PIDF);
    PIDFCoefficients appliedPidf = testMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

    if (isStopRequested()) return;

    runtime.reset();

    while (opModeIsActive()) {
      if (gamepad1.xWasPressed()) {
        running = false;
      }
      if (gamepad1.aWasPressed() && !running) {
        running = true;
        runtime.reset();
        time_recorded = false;
        time2_recorded = false;
      }
      if (gamepad1.yWasPressed()) {
        runtime.reset();
        maxMotorVelocity = 0;
        maxMotor2Velocity = 0;
        time_recorded = false;
        time2_recorded = false;
        timeTaken = "0";
        timeTaken2 = "0";
      }

      testMotor.setDirection(
          shooter1forward ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
      testMotor2.setDirection(
          shooter2forward ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

      boolean motor1Enabled = running && enabled != EnabledMotors.SHOOTER2;
      boolean motor2Enabled = running && enabled != EnabledMotors.SHOOTER1;
      drive(testMotor, motor1Enabled);
      drive(testMotor2, motor2Enabled);

      double motor_velocity = testMotor.getVelocity();
      double motor2_velocity = testMotor2.getVelocity();
      double error = targetVelocity - motor_velocity;
      double error2 = targetVelocity - motor2_velocity;

      maxMotorVelocity = Math.max(maxMotorVelocity, motor_velocity);
      maxMotor2Velocity = Math.max(maxMotor2Velocity, motor2_velocity);

      if (motor1Enabled && motor_velocity > (targetVelocity * .95) && !time_recorded) {
        timeTaken = runtime.toString();
        time_recorded = true;
      }

      if (motor2Enabled && motor2_velocity > (targetVelocity * .95) && !time2_recorded) {
        timeTaken2 = runtime.toString();
        time2_recorded = true;
      }

      telemetryM.addData("Enabled", enabled);
      telemetryM.addData("Running", running);
      telemetryM.addData("Velocity PIDF", appliedPidf);
      telemetryM.addData("Target Velocity", targetVelocity);
      telemetryM.addData("Motor Velocity", motor_velocity);
      telemetryM.addData("Motor Velocity Motor 2", motor2_velocity);
      telemetryM.addData("Motor error", error);
      telemetryM.addData("Motor error Motor 2", error2);
      telemetryM.addData("Max Velocity", maxMotorVelocity);
      telemetryM.addData("Max Velocity Motor 2", maxMotor2Velocity);
      telemetryM.addData("time Taken", timeTaken);
      telemetryM.addData("time Taken2", timeTaken2);
      telemetryM.update();
    }
  }

  /**
   * Runs the motor at {@link #targetVelocity}, or cuts power and lets it coast. A disabled motor is
   * switched to RUN_WITHOUT_ENCODER because in RUN_USING_ENCODER even power 0 is a velocity-0
   * command that would hold the shared flywheel back.
   */
  private static void drive(DcMotorEx motor, boolean spin) {
    if (spin) {
      if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      }
      // A velocity (ticks/s), not a power: setPower(1000) clipped to full power, so this ran at the
      // motor's top speed and the reported time-to-95% of targetVelocity meant nothing.
      motor.setVelocity(targetVelocity);
    } else {
      if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }
      motor.setPower(0);
    }
  }
}
