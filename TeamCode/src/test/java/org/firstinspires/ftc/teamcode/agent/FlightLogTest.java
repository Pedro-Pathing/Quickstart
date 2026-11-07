package org.firstinspires.ftc.teamcode.agent;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** {@link FlightLog}: the flight recorder's files and its directory housekeeping. */
class FlightLogTest {

  @TempDir Path tmp;

  private static String read(File f) throws IOException {
    return new String(Files.readAllBytes(f.toPath()), StandardCharsets.UTF_8);
  }

  @Test
  void writesHeaderRowsWithASequenceAndMetaOnClose() throws IOException {
    File dir = tmp.resolve("f1").toFile();
    FlightLog log = new FlightLog(dir, List.of("shooter.vel", "shooter.power"), 100);
    assertTrue(log.addRow(0, new float[] {1400f, 0.5f}));
    assertTrue(log.addRow(7, new float[] {1390f, Float.NaN}));
    log.flush();
    assertEquals(
        "t_ms,loop_seq,shooter.vel,shooter.power\n0,0,1400,0.5\n7,1,1390,\n",
        read(new File(dir, FlightLog.HARDWARE)),
        "rows are on disk after a flush, before close");

    JsonObject meta = new JsonObject();
    meta.addProperty("rows", log.rows());
    log.close(meta);
    assertEquals(
        2,
        new JsonParser()
            .parse(read(new File(dir, FlightLog.META)))
            .getAsJsonObject()
            .get("rows")
            .getAsInt());
    assertFalse(log.addRow(9, new float[] {1f, 1f}), "closed logs take no rows");
    log.close(meta); // idempotent
  }

  @Test
  void stopsAtMaxRowsAndMarksTruncated() throws IOException {
    FlightLog log = new FlightLog(tmp.resolve("f2").toFile(), List.of("x"), 2);
    assertTrue(log.addRow(0, new float[] {1}));
    assertTrue(log.addRow(1, new float[] {2}));
    assertFalse(log.truncated());
    assertFalse(log.addRow(2, new float[] {3}));
    assertTrue(log.truncated());
    assertEquals(2, log.rows());
  }

  @Test
  void eventsAndTelemetryAreJsonLines() throws IOException {
    File dir = tmp.resolve("f3").toFile();
    FlightLog log = new FlightLog(dir, List.of("x"), 10);
    JsonObject data = new JsonObject();
    data.addProperty("opmode", "TeleOp");
    log.event(12, "opmode_start", data);
    TelemetryLog t = new TelemetryLog(10);
    t.add(5, "TeleOp", List.of(new TelemetryLog.Line("vel", "1400")));
    log.addTelemetry(t.since(0, 10));
    log.flush();

    JsonObject e =
        new JsonParser().parse(read(new File(dir, FlightLog.EVENTS)).trim()).getAsJsonObject();
    assertEquals("opmode_start", e.get("event").getAsString());
    assertEquals(12, e.get("t_ms").getAsLong());
    assertEquals("TeleOp", e.get("opmode").getAsString());
    assertFalse(data.has("event"), "the caller's object is not modified");

    JsonObject tel =
        new JsonParser().parse(read(new File(dir, FlightLog.TELEMETRY)).trim()).getAsJsonObject();
    assertEquals(1, tel.get("seq").getAsLong());
  }

  @Test
  void onlyPlainNamesAreSafe() {
    assertTrue(FlightLog.safeName("20261105-120000-TeleOp"));
    assertTrue(FlightLog.safeName("hardware.csv"));
    assertFalse(FlightLog.safeName("../etc"));
    assertFalse(FlightLog.safeName("a/b"));
    assertFalse(FlightLog.safeName(".hidden"));
    assertFalse(FlightLog.safeName(""));
    assertFalse(FlightLog.safeName("a..b"));
  }

  @Test
  void newDirsAreTimestampedSluggedAndUnique() throws IOException {
    File root = tmp.toFile();
    File a = FlightLog.newDir(root, "Shooter Characterization!", 0);
    assertTrue(a.getName().endsWith("-Shooter-Characterization"), a.getName());
    assertTrue(FlightLog.safeName(a.getName()));
    assertTrue(a.mkdirs());
    File b = FlightLog.newDir(root, "Shooter Characterization!", 0);
    assertNotEquals(a, b, "same second, same label: still a new directory");
    assertTrue(FlightLog.newDir(root, "///", 0).getName().endsWith("-flight"));
  }

  @Test
  void pruneKeepsTheNewestAndNeverTheRunningFlight() throws IOException {
    File root = tmp.toFile();
    for (String name : List.of("20260101-000000-a", "20260102-000000-b", "20260103-000000-c")) {
      File d = new File(root, name);
      assertTrue(d.mkdirs());
      Files.write(
          new File(d, FlightLog.HARDWARE).toPath(), "t_ms\n".getBytes(StandardCharsets.UTF_8));
    }
    File running = new File(root, "20260101-000000-a");
    List<File> deleted = FlightLog.prune(root, 1, running);
    assertEquals(List.of(new File(root, "20260102-000000-b")), deleted);
    assertEquals(
        List.of("20260103-000000-c", "20260101-000000-a"),
        FlightLog.flights(root).stream().map(File::getName).toList());
  }
}
