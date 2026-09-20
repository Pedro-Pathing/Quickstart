package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.MechanismControls;
import org.firstinspires.ftc.teamcode.control.MechanismControls.Actions;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/** Initializes and tests only the intake motor. */
@TeleOp(name = "Test: Intake", group = "Diagnostics")
public final class IntakeTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
    MechanismControls controls = new MechanismControls();

    telemetry.addLine("A toggles intake on/off; Y toggles forward/reverse.");
    telemetry.update();
    waitForStart();

    try {
      while (opModeIsActive()) {
        Actions actions = controls.read(gamepad1, gamepad2);
        if (actions.shouldToggleIntakeEnabled()) {
          intake.toggleEnabled();
        }
        if (actions.shouldToggleIntakeDirection()) {
          intake.toggleDirection();
        }

        telemetry.addData("Intake", intake.isEnabled() ? "ON" : "OFF");
        telemetry.addData("Direction", intake.getDirection());
        telemetry.update();
        idle();
      }
    } finally {
      intake.stop();
    }
  }
}
