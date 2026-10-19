package org.firstinspires.ftc.teamcode.telemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Displays the configured pod conversion without initializing odometry hardware. */
public final class OdometryTelemetry {
  private static final String COEFFICIENT_LABEL =
      "Ticks to Inches Coefficient (in/tick, configured)";

  private OdometryTelemetry() {}

  public static void add(Telemetry telemetry) {
    telemetry.addData(COEFFICIENT_LABEL, "%.8f", RobotConfig.Pinpoint.INCHES_PER_TICK);
  }

  public static void add(TelemetryManager telemetry) {
    telemetry.addData(COEFFICIENT_LABEL, RobotConfig.Pinpoint.INCHES_PER_TICK);
  }

  public static void add(TelemetryPacket packet) {
    packet.put(COEFFICIENT_LABEL, RobotConfig.Pinpoint.INCHES_PER_TICK);
  }
}
