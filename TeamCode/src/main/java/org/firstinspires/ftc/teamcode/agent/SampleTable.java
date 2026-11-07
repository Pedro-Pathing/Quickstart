package org.firstinspires.ftc.teamcode.agent;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Fixed-column, bounded table of timestamped samples, written as CSV. Values are floats (exact for
 * encoder counts up to 2^24 and for every 16-bit hub reading); NaN means "not read" and is an empty
 * CSV cell. Thread-safe: the recorder thread appends while HTTP threads read.
 */
public final class SampleTable {

  private final List<String> columns;
  private final int maxRows;
  private final List<float[]> rows = new ArrayList<>();
  private long[] times = new long[256];
  private boolean full;

  public SampleTable(List<String> columns, int maxRows) {
    if (maxRows < 1) throw new IllegalArgumentException("maxRows must be positive");
    this.columns = List.copyOf(columns);
    this.maxRows = maxRows;
  }

  public List<String> columns() {
    return columns;
  }

  /**
   * Appends one row taken at {@code tMs}. Returns false, and marks the table full, once {@code
   * maxRows} rows are held: a recording keeps its start rather than its end.
   */
  public synchronized boolean add(long tMs, float[] values) {
    if (values.length != columns.size()) {
      throw new IllegalArgumentException(
          "row has " + values.length + " values for " + columns.size() + " columns");
    }
    if (rows.size() >= maxRows) {
      full = true;
      return false;
    }
    if (rows.size() == times.length) {
      times = Arrays.copyOf(times, times.length * 2);
    }
    times[rows.size()] = tMs;
    rows.add(values.clone());
    return true;
  }

  public synchronized int size() {
    return rows.size();
  }

  public synchronized boolean isFull() {
    return full;
  }

  /** Header {@code t_ms,<columns>} then one line per row. */
  public synchronized void writeCsv(Appendable out) throws IOException {
    out.append("t_ms");
    for (String c : columns) {
      out.append(',').append(c);
    }
    out.append('\n');
    for (int r = 0; r < rows.size(); r++) {
      out.append(Long.toString(times[r]));
      for (float v : rows.get(r)) {
        out.append(',');
        if (!Float.isNaN(v)) out.append(format(v));
      }
      out.append('\n');
    }
  }

  /** Integers without a decimal point; everything else as the shortest exact float string. */
  static String format(float v) {
    if (v == Math.rint(v) && Math.abs(v) < 1e9f) {
      return Long.toString((long) v);
    }
    return Float.toString(v);
  }
}
