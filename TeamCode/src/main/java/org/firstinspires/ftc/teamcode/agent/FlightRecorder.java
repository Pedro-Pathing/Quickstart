package org.firstinspires.ftc.teamcode.agent;

import androidx.annotation.Nullable;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.util.RobotLog;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.locks.LockSupport;

/**
 * Records what the running OpMode read, as it read it: a {@link HardwareRecorder} in {@link
 * HardwareRecorder.Source#PASSIVE} mode is checked {@code flight_poll_hz} times a second, and each
 * time the OpMode refreshed a hub's bulk cache or a Pinpoint updated, one row goes to the flight's
 * {@code hardware.csv}. Telemetry updates and bridge events (OpMode transitions, gamepad scripts,
 * config sets) go alongside, and everything is flushed every {@code flight_flush_ms}.
 *
 * <p>It sends nothing to the hubs, so recording a match changes nothing about it.
 */
final class FlightRecorder {

  private static final String TAG = "AgentBridge";

  private final HardwareRecorder source;
  private final FlightLog log;
  private final TelemetryLog telemetry;
  private final String label;
  private final boolean automatic;
  private final long startNanos = System.nanoTime();
  private final long startedAtMs = System.currentTimeMillis();
  private final JsonObject layout;
  private final double pollHz;
  private final long flushMs;
  private final Thread thread;
  private volatile boolean running = true;
  @Nullable private volatile String error;
  @Nullable private volatile String endReason;
  private long telemetrySeq;

  private FlightRecorder(
      HardwareRecorder source,
      FlightLog log,
      TelemetryLog telemetry,
      String label,
      boolean automatic,
      double pollHz,
      long flushMs) {
    this.source = source;
    this.log = log;
    this.telemetry = telemetry;
    this.label = label;
    this.automatic = automatic;
    this.pollHz = pollHz;
    this.flushMs = flushMs;
    this.layout = source.layout();
    this.telemetrySeq = telemetry.latestSeq();
    this.thread = new Thread(this::run, "AgentBridge-Flight");
    thread.setDaemon(true);
  }

  /** Creates the flight's directory under {@code root}, prunes old flights and starts. */
  static FlightRecorder start(
      HardwareRecorder passiveSource,
      TelemetryLog telemetry,
      File root,
      String label,
      boolean automatic,
      double pollHz,
      long flushMs,
      int maxRows,
      int keep)
      throws IOException {
    File dir = FlightLog.newDir(root, label, System.currentTimeMillis());
    FlightLog log = new FlightLog(dir, passiveSource.columns(), maxRows);
    FlightLog.prune(root, keep, dir);
    FlightRecorder f =
        new FlightRecorder(passiveSource, log, telemetry, label, automatic, pollHz, flushMs);
    log.writeMeta(f.meta());
    f.thread.start();
    RobotLog.ii(TAG, "flight %s started (%s)", dir.getName(), automatic ? "auto" : "agent");
    return f;
  }

  private long nowMs() {
    return (System.nanoTime() - startNanos) / 1_000_000L;
  }

  private void run() {
    long period = (long) (1e9 / pollHz);
    long nextFlush = System.nanoTime() + flushMs * 1_000_000L;
    while (running) {
      try {
        float[] row = source.sampleIfChanged();
        if (row != null) log.addRow(nowMs(), row);
        if (System.nanoTime() >= nextFlush) {
          drainTelemetry();
          log.flush();
          nextFlush = System.nanoTime() + flushMs * 1_000_000L;
        }
      } catch (IOException | RuntimeException e) {
        error = e.toString();
        RobotLog.ee(TAG, e, "flight recorder");
        running = false;
        endReason = "error";
        break;
      }
      LockSupport.parkNanos(period);
    }
  }

  private void drainTelemetry() throws IOException {
    var entries = telemetry.since(telemetrySeq, 10_000);
    if (!entries.isEmpty()) {
      log.addTelemetry(entries);
      telemetrySeq = entries.get(entries.size() - 1).seq();
    }
  }

  /** Records a bridge event (OpMode transition, gamepad script, config set). */
  void event(String what, JsonObject data) {
    log.event(nowMs(), what, data);
  }

  /** Stops sampling, writes the last rows and telemetry, and closes the files. */
  synchronized JsonObject stop(String reason) {
    if (endReason == null) endReason = reason;
    running = false;
    try {
      thread.join(1000);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    try {
      drainTelemetry();
      log.close(meta());
    } catch (IOException e) {
      error = e.toString();
    }
    RobotLog.ii(TAG, "flight %s stopped: %s, %d rows", id(), endReason, log.rows());
    return meta();
  }

  boolean isRunning() {
    return running;
  }

  boolean isAutomatic() {
    return automatic;
  }

  String id() {
    return log.dir().getName();
  }

  JsonObject meta() {
    JsonObject m = new JsonObject();
    m.addProperty("id", id());
    m.addProperty("label", label);
    m.addProperty("started_by", automatic ? "flight_auto" : "agent");
    m.addProperty("started_at_ms", startedAtMs);
    m.addProperty("duration_ms", nowMs());
    m.addProperty("recording", running);
    m.addProperty("rows", log.rows());
    m.addProperty("truncated", log.truncated());
    m.addProperty("poll_hz", pollHz);
    m.addProperty(
        "source",
        "passive: the values the OpMode read (hub bulk caches, Pinpoint driver fields); one row"
            + " per new read");
    String reason = endReason;
    if (reason != null) m.addProperty("end_reason", reason);
    String err = error;
    if (err != null) m.addProperty("error", err);
    m.add("columns", layout.get("columns"));
    m.add("devices", layout.get("devices"));
    m.add("skipped", layout.get("skipped"));
    return m;
  }
}
