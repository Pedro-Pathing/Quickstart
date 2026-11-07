package org.firstinspires.ftc.teamcode.agent;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.util.RobotLog;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;

/**
 * Records every telemetry update an OpMode sends to the Driver Station, without the OpMode's
 * cooperation.
 *
 * <p>{@code Telemetry.update()} ends in {@code OpMode.internalUpdateTelemetryNow}, which hands the
 * message to the OpMode's {@code internalOpModeServices}. The tap replaces that field with a
 * wrapper that copies the message into a {@link TelemetryLog} and forwards it unchanged, so the
 * Driver Station sees exactly what it did before. Install it from {@code onOpModePreInit}: the SDK
 * sets the field just before notifying.
 *
 * <p>Telemetry is throttled by the SDK ({@code Telemetry.setMsTransmissionInterval}, 250 ms by
 * default), so it is what the drivers see, not a loop-rate signal; use the hardware recorder for
 * that.
 */
final class TelemetryTap {

  private static final String TAG = "AgentBridge";

  private TelemetryTap() {}

  static void install(OpMode op, String opModeName, TelemetryLog log) {
    try {
      Field f = servicesField(op);
      Object current = f.get(op);
      if (current instanceof Recording || current == null) return;
      f.set(op, new Recording((OpModeServices) current, log, opModeName));
    } catch (ReflectiveOperationException | RuntimeException e) {
      RobotLog.ee(TAG, e, "telemetry tap not installed");
    }
  }

  private static Field servicesField(OpMode op) throws NoSuchFieldException {
    for (Class<?> c = op.getClass(); c != null; c = c.getSuperclass()) {
      if (c.getName().equals("com.qualcomm.robotcore.eventloop.opmode.OpModeInternal")) {
        Field f = c.getDeclaredField("internalOpModeServices");
        f.setAccessible(true);
        return f;
      }
    }
    throw new NoSuchFieldException("internalOpModeServices");
  }

  /** Lines in display order, with the SDK's ordering prefix stripped from each caption. */
  static List<TelemetryLog.Line> lines(Map<String, String> data) {
    List<TelemetryLog.Line> out = new ArrayList<>(data.size());
    for (Map.Entry<String, String> e : data.entrySet()) {
      String key = e.getKey() == null ? "" : e.getKey();
      int start = 0;
      while (start < key.length() && key.charAt(start) < ' ') start++;
      out.add(new TelemetryLog.Line(key.substring(start), e.getValue()));
    }
    return out;
  }

  private static final class Recording implements OpModeServices {
    private final OpModeServices inner;
    private final TelemetryLog log;
    private final String opModeName;

    Recording(OpModeServices inner, TelemetryLog log, String opModeName) {
      this.inner = inner;
      this.log = log;
      this.opModeName = opModeName;
    }

    @Override
    public void refreshUserTelemetry(TelemetryMessage message, double sInterval) {
      try {
        log.add(System.currentTimeMillis(), opModeName, lines(message.getDataStrings()));
      } catch (RuntimeException e) {
        RobotLog.ee(TAG, e, "telemetry tap failed to record");
      }
      inner.refreshUserTelemetry(message, sInterval);
    }

    @Override
    public void requestOpModeStop(OpMode opMode) {
      inner.requestOpModeStop(opMode);
    }
  }
}
