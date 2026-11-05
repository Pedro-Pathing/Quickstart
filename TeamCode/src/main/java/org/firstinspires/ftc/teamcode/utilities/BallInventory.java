package org.firstinspires.ftc.teamcode.utilities;

/**
 * How many SCORING ELEMENTS the robot CONTROLS, so the intake can stop itself at the G407 limit of
 * four.
 *
 * <p>Elements are counted in by an intake-entry sensor (debounced, one count per blocked-then-clear
 * pass) and out by shots the flywheel dip detector confirms. Without an entry sensor the count is
 * only known after it has been set explicitly (the four pre-loads at the start of AUTO) and becomes
 * unknown the moment the intake runs. An unknown count never reports full, so a robot without the
 * sensor behaves exactly as before rather than refusing to intake.
 *
 * <p>A software count cannot stop a fifth ball that is already being pulled in: the intake still
 * needs a mechanical stop. This only removes the most common cause, an intake left running.
 */
public final class BallInventory {

  private final int capacity;
  private final boolean sensed;

  private int count = 0;
  private boolean known = true;

  private boolean sensorBlocked = false;
  private boolean entryCounted = false;
  private long blockedSinceMs = 0;

  public BallInventory(int capacity, boolean sensed) {
    if (capacity < 1) {
      throw new IllegalArgumentException("capacity must be at least 1");
    }
    this.capacity = capacity;
    this.sensed = sensed;
  }

  public int capacity() {
    return capacity;
  }

  /** True when an intake-entry sensor is fitted and counting. */
  public boolean isSensed() {
    return sensed;
  }

  /** True when {@link #count()} is trustworthy. */
  public boolean isKnown() {
    return known;
  }

  /** Elements controlled, or the last known value when {@link #isKnown()} is false. */
  public int count() {
    return count;
  }

  /** Only ever true when the count is known; an unknown inventory never blocks the intake. */
  public boolean isFull() {
    return isFull(known, count, capacity);
  }

  /**
   * G407: whether a collect may keep the intake running. Exactly {@code !isFull}: an unknown count
   * never blocks the intake, a known count at or over capacity always does.
   */
  /*@ normal_behavior
  @   ensures known && count >= capacity ==> !\result;
  @   ensures !known ==> \result;
  @   ensures known && count < capacity ==> \result;
  @ pure
  @*/
  public static boolean mayCollect(boolean known, int count, int capacity) {
    return !isFull(known, count, capacity);
  }

  /*@ normal_behavior
  @   ensures \result <==> (known && count >= capacity);
  @ pure
  @*/
  static boolean isFull(boolean known, int count, int capacity) {
    return known && count >= capacity;
  }

  /**
   * The largest count the inventory can hold: one over {@code capacity}. The software cannot stop
   * an element already being pulled in, so a fifth ball (G407 violation) must be representable, or
   * the count would read "full" while the robot actually controls too many. Saturates instead of
   * overflowing for a huge capacity.
   */
  /*@ normal_behavior
  @   ensures capacity < Integer.MAX_VALUE ==> \result == capacity + 1;
  @   ensures capacity == Integer.MAX_VALUE ==> \result == Integer.MAX_VALUE;
  @   ensures \result >= capacity;
  @ pure
  @*/
  static int ceiling(int capacity) {
    return capacity == Integer.MAX_VALUE ? capacity : capacity + 1;
  }

  /** {@code elements} limited to {@code [0, ceiling(capacity)]}. */
  /*@ normal_behavior
  @   ensures \result >= 0;
  @   ensures capacity >= 1 ==> \result <= ceiling(capacity);
  @   ensures capacity >= 1 && 0 <= elements && elements <= ceiling(capacity)
  @           ==> \result == elements;
  @ pure
  @*/
  static int clampCount(int elements, int capacity) {
    return Math.max(0, Math.min(elements, ceiling(capacity)));
  }

  /** The count after one confirmed shot: down by one, never below zero. */
  /*@ normal_behavior
  @   ensures count > 0 ==> \result == count - 1;
  @   ensures count <= 0 ==> \result == count;
  @ pure
  @*/
  static int afterShot(int count) {
    return count > 0 ? count - 1 : count;
  }

  /**
   * The count after an element passed the entry sensor. Unknown stays unknown-valued (unchanged).
   * Inward stops at {@link #ceiling}, outward at zero.
   */
  /*@ normal_behavior
  @   ensures !known ==> \result == count;
  @   ensures known && outward ==> \result == (count > 0 ? count - 1 : 0);
  @   ensures known && !outward && count < ceiling(capacity) ==> \result == count + 1;
  @   ensures known && !outward && count >= ceiling(capacity) ==> \result == ceiling(capacity);
  @   ensures known && 0 <= count && count <= ceiling(capacity)
  @           ==> 0 <= \result && \result <= ceiling(capacity);
  @ pure
  @*/
  static int afterEntry(int count, boolean known, boolean outward, int capacity) {
    if (!known) {
      return count;
    }
    if (outward) {
      return count > 0 ? count - 1 : 0;
    }
    int top = ceiling(capacity);
    return count >= top ? top : count + 1;
  }

  /**
   * Whether the sensor has been blocked for at least {@code debounceMs} at {@code nowMs}, without
   * the wrap-around of a plain {@code nowMs - sinceMs}: it is {@code now - since >= debounce} in
   * unbounded arithmetic. OpenJML proves the cases below (it cannot state the unbounded form for
   * longs); the unit test checks the whole space against a BigInteger oracle.
   */
  /*@ normal_behavior
  @   ensures nowMs < sinceMs && debounceMs >= 0 ==> !\result;
  @   ensures nowMs >= sinceMs && debounceMs <= 0 ==> \result;
  @   ensures 0 <= sinceMs && sinceMs <= nowMs ==> (\result <==> nowMs - sinceMs >= debounceMs);
  @   ensures nowMs < 0 && sinceMs <= nowMs ==> (\result <==> nowMs - sinceMs >= debounceMs);
  @   ensures sinceMs < 0 && nowMs >= 0 && debounceMs > 0
  @           ==> (\result <==> nowMs >= debounceMs + sinceMs);
  @   ensures sinceMs < 0 && nowMs >= 0 && debounceMs <= 0 ==> \result;
  @   ensures nowMs < 0 && sinceMs >= 0 && debounceMs < 0
  @           ==> (\result <==> nowMs >= debounceMs + sinceMs);
  @   ensures nowMs < 0 && sinceMs >= 0 && debounceMs >= 0 ==> !\result;
  @   ensures sinceMs == Long.MIN_VALUE && nowMs == Long.MAX_VALUE ==> \result;
  @ pure
  @*/
  static boolean elapsed(long nowMs, long sinceMs, long debounceMs) {
    if (nowMs >= sinceMs) {
      if (sinceMs >= 0 || nowMs < 0) {
        return nowMs - sinceMs >= debounceMs; // same sign: cannot overflow
      }
      // since < 0 <= now: the true gap is positive and may not fit a long; compare sums instead.
      return debounceMs <= 0 || nowMs >= debounceMs + sinceMs;
    }
    if (nowMs >= 0 || sinceMs < 0) {
      return nowMs - sinceMs >= debounceMs; // same sign: cannot overflow
    }
    // now < 0 <= since: the true gap is negative and may not fit a long.
    return debounceMs < 0 && nowMs >= debounceMs + sinceMs;
  }

  /** When the current block started: kept while it lasts, {@code nowMs} on its first reading. */
  /*@ normal_behavior
  @   ensures wasBlocked ==> \result == sinceMs;
  @   ensures !wasBlocked ==> \result == nowMs;
  @ pure
  @*/
  static long blockStart(boolean wasBlocked, long sinceMs, long nowMs) {
    return wasBlocked ? sinceMs : nowMs;
  }

  /**
   * Whether this reading counts an element: the sensor is blocked, this block has not counted one
   * yet, and it has lasted the debounce. So one block counts at most once.
   */
  /*@ normal_behavior
  @   ensures !blocked || alreadyCounted ==> !\result;
  @   ensures \result ==> blocked && !alreadyCounted && elapsed(nowMs, sinceMs, debounceMs);
  @   ensures blocked && !alreadyCounted && elapsed(nowMs, sinceMs, debounceMs) ==> \result;
  @ pure
  @*/
  static boolean countsNow(
      boolean blocked, boolean alreadyCounted, long nowMs, long sinceMs, long debounceMs) {
    return blocked && !alreadyCounted && elapsed(nowMs, sinceMs, debounceMs);
  }

  public boolean isEmpty() {
    return known && count <= 0;
  }

  /** Sets a count observed some other way: pre-loads, or the operator correcting it. */
  public void set(int elements) {
    count = clampCount(elements, capacity);
    known = true;
  }

  /** Restore a handoff total; -1 is unknown even when an entry sensor is fitted. */
  public void restore(int elements) {
    set(Math.max(0, elements));
    known = elements >= 0;
  }

  /** The intake ran without a sensor to count what it took in. */
  public void markUnknown() {
    if (!sensed) {
      known = false;
    }
  }

  /** One element left the robot through the shooter. */
  public void onShot() {
    count = afterShot(count);
  }

  /** {@link #updateEntrySensor(boolean, long, long, boolean)} for elements coming in. */
  public boolean updateEntrySensor(boolean blocked, long nowMs, long debounceMs) {
    return updateEntrySensor(blocked, nowMs, debounceMs, false);
  }

  /**
   * Feeds one raw entry-sensor reading. An element is counted once it has blocked the sensor for
   * {@code debounceMs}; it cannot be counted again until the sensor clears.
   *
   * @param outward true while the intake is ejecting, so a pass removes an element instead
   * @return true if this reading counted an element passing the sensor
   */
  public boolean updateEntrySensor(boolean blocked, long nowMs, long debounceMs, boolean outward) {
    if (!blocked) {
      sensorBlocked = false;
      entryCounted = false;
      return false;
    }
    blockedSinceMs = blockStart(sensorBlocked, blockedSinceMs, nowMs);
    sensorBlocked = true;
    if (countsNow(true, entryCounted, nowMs, blockedSinceMs, debounceMs)) {
      entryCounted = true;
      count = afterEntry(count, known, outward, capacity);
      return true;
    }
    return false;
  }
}
