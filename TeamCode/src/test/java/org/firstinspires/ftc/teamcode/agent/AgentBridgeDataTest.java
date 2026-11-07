package org.firstinspires.ftc.teamcode.agent;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** The agent bridge's buffers: {@link SampleTable}, {@link TelemetryLog}, telemetry captions. */
class AgentBridgeDataTest {

  @Test
  void csvHasHeaderRowsAndEmptyCellsForUnread() throws IOException {
    SampleTable t = new SampleTable(List.of("a.pos", "a.power"), 10);
    assertTrue(t.add(0, new float[] {120f, 0.5f}));
    assertTrue(t.add(20, new float[] {-7f, Float.NaN}));
    StringBuilder out = new StringBuilder();
    t.writeCsv(out);
    assertEquals("t_ms,a.pos,a.power\n0,120,0.5\n20,-7,\n", out.toString());
  }

  @Test
  void tableKeepsItsStartWhenFull() throws IOException {
    SampleTable t = new SampleTable(List.of("x"), 3);
    for (int i = 0; i < 3; i++) assertTrue(t.add(i, new float[] {i}));
    assertFalse(t.isFull());
    assertFalse(t.add(3, new float[] {3}));
    assertTrue(t.isFull());
    assertEquals(3, t.size());
    StringBuilder out = new StringBuilder();
    t.writeCsv(out);
    assertTrue(out.toString().endsWith("2,2\n"));
  }

  @Test
  void tableGrowsPastItsInitialTimeBuffer() {
    SampleTable t = new SampleTable(List.of("x"), 5000);
    for (int i = 0; i < 5000; i++) assertTrue(t.add(i, new float[] {i}));
    assertEquals(5000, t.size());
  }

  @Test
  void rowWidthMustMatchColumns() {
    SampleTable t = new SampleTable(List.of("x", "y"), 3);
    assertThrows(IllegalArgumentException.class, () -> t.add(0, new float[] {1}));
  }

  @Test
  void floatFormatting() {
    assertEquals("1400", SampleTable.format(1400f));
    assertEquals("-3", SampleTable.format(-3f));
    assertEquals("0.25", SampleTable.format(0.25f));
    assertEquals(16777216f, Float.parseFloat(SampleTable.format(16777216f)));
  }

  @Test
  void telemetryLogPollsBySequenceAndDropsOldest() {
    TelemetryLog log = new TelemetryLog(3);
    assertEquals(0, log.latestSeq());
    assertNull(log.latest());
    for (int i = 1; i <= 5; i++) {
      assertEquals(i, log.add(i * 100L, "Op", List.of(new TelemetryLog.Line("n", "" + i))));
    }
    assertEquals(5, log.latestSeq());
    List<TelemetryLog.Entry> all = log.since(0, 100);
    assertEquals(List.of(3L, 4L, 5L), all.stream().map(TelemetryLog.Entry::seq).toList());
    assertEquals(List.of(5L), log.since(4, 100).stream().map(TelemetryLog.Entry::seq).toList());
    assertEquals(1, log.since(0, 1).size());
    assertEquals("5", log.latest().lines().get(0).value());
  }

  @Test
  void telemetryCaptionsLoseTheSdkOrderingPrefix() {
    Map<String, String> data = new LinkedHashMap<>();
    data.put("\u0000\u0001Flywheel", "1400");
    data.put("plain", "x");
    List<TelemetryLog.Line> lines = TelemetryTap.lines(data);
    assertEquals("Flywheel", lines.get(0).caption());
    assertEquals("1400", lines.get(0).value());
    assertEquals("plain", lines.get(1).caption());
  }
}
