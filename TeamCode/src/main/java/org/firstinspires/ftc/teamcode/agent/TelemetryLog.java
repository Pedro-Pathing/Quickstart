package org.firstinspires.ftc.teamcode.agent;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

/**
 * Bounded log of the telemetry updates an OpMode sent to the Driver Station, numbered so a client
 * can poll for only what it has not seen. Thread-safe: written from the OpMode's thread, read from
 * HTTP threads.
 */
public final class TelemetryLog {

  public record Line(String caption, String value) {}

  public record Entry(long seq, long tMs, String opMode, List<Line> lines) {}

  private final int capacity;
  private final Deque<Entry> entries = new ArrayDeque<>();
  private long nextSeq = 1;

  public TelemetryLog(int capacity) {
    if (capacity < 1) throw new IllegalArgumentException("capacity must be positive");
    this.capacity = capacity;
  }

  /** Appends an update and returns its sequence number; the oldest entry is dropped when full. */
  public synchronized long add(long tMs, String opMode, List<Line> lines) {
    long seq = nextSeq++;
    entries.addLast(new Entry(seq, tMs, opMode, List.copyOf(lines)));
    while (entries.size() > capacity) {
      entries.removeFirst();
    }
    return seq;
  }

  /**
   * Up to {@code max} entries with a sequence number greater than {@code afterSeq}, oldest first.
   */
  public synchronized List<Entry> since(long afterSeq, int max) {
    List<Entry> out = new ArrayList<>();
    for (Entry e : entries) {
      if (e.seq() > afterSeq) {
        out.add(e);
        if (out.size() >= max) break;
      }
    }
    return out;
  }

  /** The newest entry, or null if there is none. */
  public synchronized Entry latest() {
    return entries.peekLast();
  }

  /** Sequence number of the newest entry; 0 before the first. */
  public synchronized long latestSeq() {
    return nextSeq - 1;
  }
}
