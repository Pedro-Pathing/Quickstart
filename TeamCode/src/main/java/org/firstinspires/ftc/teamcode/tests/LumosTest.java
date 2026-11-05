package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Lumos;

@TeleOp(name = "Lumos Test", group = "Test")
public class LumosTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    org.firstinspires.ftc.teamcode.utilities.OpModeUtil.reportConfig(telemetry);
    Lumos lumos = new Lumos(hardwareMap);
    waitForStart();

    if (isStopRequested()) return;

    while (opModeIsActive()) {
      lumos.setColor(Lumos.Color.RED);
      lumos.periodic();
      telemetry.addData("pos", lumos.getPosition());
      telemetry.update();
      idle();
    }
  }
}
