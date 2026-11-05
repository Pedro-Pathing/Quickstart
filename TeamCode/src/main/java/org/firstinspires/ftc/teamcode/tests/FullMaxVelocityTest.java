package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Full Max Velocity Test", group = "Test")
@Disabled
public class FullMaxVelocityTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    org.firstinspires.ftc.teamcode.utilities.OpModeUtil.reportConfig(telemetry);
    final DcMotorEx intakeF, intakeM, shooter;
    double power = 0.8;
    intakeF = hardwareMap.get(DcMotorEx.class, "intakeFront");
    intakeM = hardwareMap.get(DcMotorEx.class, "intakeMid");
    shooter = hardwareMap.get(DcMotorEx.class, "shooter");

    intakeF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // TODO: reverse motor directions if needed
    intakeF.setDirection(DcMotorSimple.Direction.REVERSE);
    intakeM.setDirection(DcMotorSimple.Direction.REVERSE);
    shooter.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    if (isStopRequested()) return;

    while (opModeIsActive()) {
      intakeF.setPower(power);
      intakeM.setPower(power);
      shooter.setPower(power);
      telemetry.addData("intake Front", intakeF.getVelocity());
      telemetry.addData("intake Mid", intakeM.getVelocity());
      telemetry.addData("shooter", shooter.getVelocity());
      telemetry.update();
    }
  }
}
