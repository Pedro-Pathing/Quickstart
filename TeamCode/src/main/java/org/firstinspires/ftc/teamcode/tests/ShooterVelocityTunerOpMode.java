package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * OpMode for tuning and monitoring shooter flywheel velocity in real-time.
 *
 * <p>Displays current shooter velocity and target velocity in ByLazar Dashboard (Panels), and
 * provides a {@code targetRPM} input field for live adjustment.
 */
@Configurable
@TeleOp(name = "Shooter Velocity Tuner", group = "Tuning")
public class ShooterVelocityTunerOpMode extends OpMode {

  /** Target RPM input field configurable via ByLazar Dashboard / Panels. */
  public static double targetRPM = 870.0;

  private Shooter shooter;
  private TelemetryManager telemetryM;
  private List<LynxModule> allHubs;

  @Override
  public void init() {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT

    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    shooter = new Shooter(hardwareMap);

    targetRPM = config.shooter.constant_rpm;

    telemetry.addData("Status", "Initialized Shooter Velocity Tuner");
    telemetry.addData("Target RPM Input", targetRPM);
    telemetry.update();
  }

  @Override
  public void loop() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }

    double maxRpm = config.shooter.max_rpm > 0 ? config.shooter.max_rpm : 1500.0;
    double targetPower = targetRPM > 0 ? targetRPM / maxRpm : 0.0;

    shooter.setTargetPower(targetPower);
    shooter.periodic();

    double currentVelocity = Math.abs(shooter.getShooterVelocity());
    double targetVelocity = shooter.getLastTargetVelocity();
    double error = shooter.getLastError();

    if (telemetryM != null) {
      telemetryM.addData("Shooter Velocity", currentVelocity);
      telemetryM.addData("Target Velocity", targetVelocity);
      telemetryM.addData("Target RPM", targetRPM);
      telemetryM.addData("Target Power", targetPower);
      telemetryM.addData("Velocity Error", error);
      telemetryM.addData("Motor Command", shooter.getLastCommand());
      telemetryM.addData("PID Term", shooter.getLastPidTerm());
      telemetryM.addData("Integral Term", shooter.getLastIntegralTerm());
      telemetryM.addData("Feedforward Term", shooter.getLastFeedforwardTerm());
      telemetryM.update();
    }

    telemetry.addData("Shooter Velocity", "%.1f ticks/s", currentVelocity);
    telemetry.addData("Target Velocity", "%.1f ticks/s", targetVelocity);
    telemetry.addData("Target RPM Input", "%.1f RPM", targetRPM);
    telemetry.addData("Target Power", "%.3f", targetPower);
    telemetry.addData("Velocity Error", "%.1f ticks/s", error);
    telemetry.addData("Motor Command", "%.3f", shooter.getLastCommand());
    telemetry.update();
  }

  @Override
  public void stop() {
    if (shooter != null) {
      shooter.stop();
    }
  }
}
