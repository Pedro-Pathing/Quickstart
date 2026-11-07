package org.firstinspires.ftc.teamcode.agent;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;

/**
 * The files of one flight: {@code hardware.csv} (header {@code t_ms,loop_seq,<columns>}, one row
 * per new read), {@code telemetry.jsonl}, {@code events.jsonl} and {@code meta.json}. Files are
 * written as the flight runs, so a reader can fetch them while it does and a crash loses only what
 * was not yet flushed. Thread-safe.
 */
final class FlightLog {

  static final String HARDWARE = "hardware.csv";
  static final String TELEMETRY = "telemetry.jsonl";
  static final String EVENTS = "events.jsonl";
  static final String META = "meta.json";
  static final List<String> FILES = List.of(HARDWARE, TELEMETRY, EVENTS, META);

  private static final Pattern SAFE = Pattern.compile("[A-Za-z0-9][A-Za-z0-9._-]{0,127}");
  private static final Gson GSON =
      new GsonBuilder().serializeSpecialFloatingPointValues().setPrettyPrinting().create();
  private static final Gson LINE = new GsonBuilder().serializeSpecialFloatingPointValues().create();

  private final File dir;
  private final int maxRows;
  private final Writer hardware;
  private final Writer telemetry;
  private final Writer events;
  private long rows;
  private boolean truncated;
  private boolean closed;

  FlightLog(File dir, List<String> columns, int maxRows) throws IOException {
    if (!dir.mkdirs() && !dir.isDirectory()) throw new IOException("cannot create " + dir);
    this.dir = dir;
    this.maxRows = maxRows;
    hardware = open(HARDWARE);
    telemetry = open(TELEMETRY);
    events = open(EVENTS);
    StringBuilder header = new StringBuilder("t_ms,loop_seq");
    for (String c : columns) header.append(',').append(c);
    hardware.write(header.append('\n').toString());
  }

  private Writer open(String name) throws IOException {
    return new BufferedWriter(
        new OutputStreamWriter(new FileOutputStream(new File(dir, name)), StandardCharsets.UTF_8));
  }

  File dir() {
    return dir;
  }

  /** Appends a row; false (and the flight marked truncated) once {@code maxRows} are written. */
  synchronized boolean addRow(long tMs, float[] values) throws IOException {
    if (closed) return false;
    if (rows >= maxRows) {
      truncated = true;
      return false;
    }
    StringBuilder line = new StringBuilder(16 + values.length * 8);
    line.append(tMs).append(',').append(rows);
    for (float v : values) {
      line.append(',');
      if (!Float.isNaN(v)) line.append(SampleTable.format(v));
    }
    hardware.write(line.append('\n').toString());
    rows++;
    return true;
  }

  synchronized void addTelemetry(List<TelemetryLog.Entry> entries) throws IOException {
    if (closed) return;
    for (TelemetryLog.Entry e : entries) {
      telemetry.write(LINE.toJson(e));
      telemetry.write('\n');
    }
  }

  synchronized void event(long tMs, String what, JsonObject data) {
    if (closed) return;
    JsonObject e = new JsonObject(); // Gson 2.8.0 has no public deepCopy
    for (java.util.Map.Entry<String, com.google.gson.JsonElement> kv : data.entrySet()) {
      e.add(kv.getKey(), kv.getValue());
    }
    e.addProperty("t_ms", tMs);
    e.addProperty("event", what);
    try {
      events.write(LINE.toJson(e));
      events.write('\n');
    } catch (IOException ignored) {
      // an event is best-effort; the rows are what matter
    }
  }

  synchronized void flush() throws IOException {
    if (closed) return;
    hardware.flush();
    telemetry.flush();
    events.flush();
  }

  synchronized void writeMeta(JsonObject meta) throws IOException {
    try (Writer w = open(META)) {
      w.write(GSON.toJson(meta));
    }
  }

  /** Flushes and closes every file, then writes {@code meta}. Idempotent. */
  synchronized void close(JsonObject meta) throws IOException {
    if (closed) return;
    flush();
    hardware.close();
    telemetry.close();
    events.close();
    closed = true;
    writeMeta(meta);
  }

  synchronized long rows() {
    return rows;
  }

  synchronized boolean truncated() {
    return truncated;
  }

  // --- directory helpers ---------------------------------------------------------------------

  /** True for names a client may ask for: no path separators, no leading dot. */
  static boolean safeName(String name) {
    return name != null && SAFE.matcher(name).matches() && !name.contains("..");
  }

  /** {@code <root>/<yyyyMMdd-HHmmss>-<label>}, with the label reduced to safe characters. */
  @SuppressWarnings("JavaUtilDate") // java.time needs Android API 26; the Control Hub runs 25
  static File newDir(File root, String label, long epochMs) {
    String slug = label.replaceAll("[^A-Za-z0-9_-]+", "-").replaceAll("^-+|-+$", "");
    if (slug.length() > 48) slug = slug.substring(0, 48);
    if (slug.isEmpty()) slug = "flight";
    String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.ROOT).format(new Date(epochMs));
    File dir = new File(root, stamp + "-" + slug);
    for (int i = 2; dir.exists(); i++) {
      dir = new File(root, stamp + "-" + slug + "-" + i);
    }
    return dir;
  }

  /** Flight directories under {@code root}, newest first (names start with their time). */
  static List<File> flights(File root) {
    File[] all = root.listFiles(f -> f.isDirectory() && safeName(f.getName()));
    if (all == null) return List.of();
    List<File> out = new ArrayList<>(Arrays.asList(all));
    out.sort((a, b) -> b.getName().compareTo(a.getName()));
    return out;
  }

  /** Deletes all but the newest {@code keep} flights, never {@code except}; returns the deleted. */
  static List<File> prune(File root, int keep, File except) {
    List<File> deleted = new ArrayList<>();
    List<File> all = flights(root);
    for (int i = keep; i < all.size(); i++) {
      File f = all.get(i);
      if (f.equals(except)) continue;
      File[] files = f.listFiles();
      if (files != null) {
        for (File child : files) {
          if (!child.delete()) break;
        }
      }
      if (f.delete()) deleted.add(f);
    }
    return deleted;
  }
}
