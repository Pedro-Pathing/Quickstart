package org.firstinspires.ftc.teamcode.game;

/**
 * Every decision {@link HiveTracker} makes, as pure functions of primitives: no clock, config or
 * state. {@link HiveTracker} holds the fields and the clock, calls these, and stores the results.
 * The up CELL is a boolean, {@code upIsA}: true when {@code HiveCell.A} is upward.
 *
 * <p>The JML contracts ({@code /*@ ... @*&#47;} comments) are proven against this exact code by
 * OpenJML (the same run as {@code TurretLaw}). They are ordinary comments to javac. Every method
 * has a contract with no precondition, so none can throw or overflow whatever the clock, camera or
 * caller gives it. Time differences never use a raw {@code a - b}: the tracker keeps {@link
 * #NEVER}, a sentinel near {@code Long.MIN_VALUE}, as a timestamp, and a clock far from zero would
 * overflow a plain subtraction. {@link #sub} saturates instead.
 */
final class HiveLaw {
  private HiveLaw() {}

  /** Timestamp meaning "this has never happened". Far enough from MIN_VALUE to subtract from. */
  static final long NEVER = Long.MIN_VALUE / 2;

  // ---------------------------------------------------------------------------------------------
  // Arithmetic that cannot overflow

  /** {@code a - b}, clamped to the range of a long instead of wrapping. */
  /*@ normal_behavior
  @   ensures (\bigint) a - (\bigint) b > (\bigint) Long.MAX_VALUE ==> \result == Long.MAX_VALUE;
  @   ensures (\bigint) a - (\bigint) b < (\bigint) Long.MIN_VALUE ==> \result == Long.MIN_VALUE;
  @   ensures (\bigint) a - (\bigint) b >= (\bigint) Long.MIN_VALUE
  @           && (\bigint) a - (\bigint) b <= (\bigint) Long.MAX_VALUE
  @           ==> (\bigint) \result == (\bigint) a - (\bigint) b;
  @ pure
  @*/
  static long sub(long a, long b) {
    if (b >= 0) {
      return a < Long.MIN_VALUE + b ? Long.MIN_VALUE : a - b;
    }
    return a > Long.MAX_VALUE + b ? Long.MAX_VALUE : a - b;
  }

  /** {@code n + 1}, stopping at the largest int rather than wrapping to a negative count. */
  /*@ normal_behavior
  @   ensures n < Integer.MAX_VALUE ==> \result == n + 1;
  @   ensures n == Integer.MAX_VALUE ==> \result == Integer.MAX_VALUE;
  @   ensures \result >= n;
  @ pure
  @*/
  static int satInc(int n) {
    return n == Integer.MAX_VALUE ? n : n + 1;
  }

  // ---------------------------------------------------------------------------------------------
  // Epoch

  /**
   * The epoch after an accepted change. Epochs are only ever compared for equality (a shot solution
   * is current iff its epoch equals the tracker's), so wrapping from the largest int to the
   * smallest is fine: it still differs from the epoch before. It would take 2^32 changes to get
   * there.
   */
  /*@ normal_behavior
  @   ensures \result != epoch;
  @   ensures epoch < Integer.MAX_VALUE ==> \result == epoch + 1;
  @   ensures epoch == Integer.MAX_VALUE ==> \result == Integer.MIN_VALUE;
  @ pure
  @*/
  static int nextEpoch(int epoch) {
    return epoch == Integer.MAX_VALUE ? Integer.MIN_VALUE : epoch + 1;
  }

  // ---------------------------------------------------------------------------------------------
  // Camera observations

  /**
   * Whether a camera frame is usable: strictly newer than the last accepted frame (so out-of-order
   * and repeated frames are dropped), not from the future, and no older than {@code maxAgeMs} (a
   * negative age limit counts as 0, as the tracker clamps it). The age test is written {@code
   * observed >= now - maxAge} with a saturating subtraction, which is the same inequality without
   * the overflow {@code now - observed} has for a frame stamped near {@code Long.MIN_VALUE}.
   */
  /*@ normal_behavior
  @   ensures \result ==> observedAtMs > lastObservationMs;
  @   ensures \result ==> observedAtMs <= nowMs;
  @   ensures \result ==> (\bigint) nowMs - (\bigint) observedAtMs <= (\bigint) Math.max(0L, maxAgeMs);
  @   ensures observedAtMs > lastObservationMs && observedAtMs <= nowMs
  @           && (\bigint) nowMs - (\bigint) observedAtMs <= (\bigint) Math.max(0L, maxAgeMs)
  @           ==> \result;
  @ pure
  @*/
  static boolean accepts(long observedAtMs, long lastObservationMs, long nowMs, long maxAgeMs) {
    return observedAtMs > lastObservationMs
        && observedAtMs <= nowMs
        && observedAtMs >= sub(nowMs, Math.max(0L, maxAgeMs));
  }

  /** The upward CELL a sighting implies: the sighted CELL if it was up, else the other one. */
  /*@ normal_behavior
  @   ensures upward ==> \result == cellIsA;
  @   ensures !upward ==> \result == !cellIsA;
  @ pure
  @*/
  static boolean impliedUpIsA(boolean cellIsA, boolean upward) {
    return cellIsA == upward;
  }

  /**
   * The count of consecutive disagreeing frames after one more accepted frame implying {@code
   * impliedUpIsA}. An agreeing frame resets it to 0. A disagreeing frame that continues the same
   * candidate adds 1; the first disagreeing frame, or one that switches candidate, starts at 1. Two
   * CELLS means a disagreeing frame always implies the CELL that is not up.
   */
  /*@ normal_behavior
  @   ensures impliedUpIsA == upIsA ==> \result == 0;
  @   ensures impliedUpIsA != upIsA && (count <= 0 || candidateUpIsA != impliedUpIsA)
  @           ==> \result == 1;
  @   ensures impliedUpIsA != upIsA && count > 0 && candidateUpIsA == impliedUpIsA
  @           ==> \result == satInc(count);
  @   ensures count >= 0 && count < Integer.MAX_VALUE && impliedUpIsA != upIsA
  @           && candidateUpIsA == impliedUpIsA && count > 0
  @           ==> \result == count + 1;
  @ pure
  @*/
  static int nextCandidateCount(
      boolean impliedUpIsA, boolean upIsA, boolean candidateUpIsA, int count) {
    if (impliedUpIsA == upIsA) {
      return 0;
    }
    if (count <= 0 || candidateUpIsA != impliedUpIsA) {
      return 1;
    }
    return satInc(count);
  }

  /** Whether enough consecutive disagreeing frames have arrived to accept the new state. */
  /*@ normal_behavior
  @   ensures \result <==> count >= confirmFrames;
  @ pure
  @*/
  static boolean confirmed(int count, int confirmFrames) {
    return count >= confirmFrames;
  }

  /**
   * Whether a vision-detected change is the HIVE rocking back inside the settle window of the
   * vision change before it: one stopped TIP, not two. Needs a TIP to undo.
   */
  /*@ normal_behavior
  @   ensures \result <==> lastChangeWasVision && tips > 0 && !settled(nowMs, lastChangeMs, settleMs);
  @ pure
  @*/
  static boolean rockedBack(
      boolean lastChangeWasVision, long nowMs, long lastChangeMs, long settleMs, int tips) {
    return lastChangeWasVision && tips > 0 && !settled(nowMs, lastChangeMs, settleMs);
  }

  /** TIP count after a vision-detected change: one fewer if it was a rock-back, else one more. */
  /*@ normal_behavior
  @   ensures rockedBack && tips > 0 ==> \result == tips - 1;
  @   ensures !(rockedBack && tips > 0) ==> \result == satInc(tips);
  @   ensures tips >= 0 ==> \result >= 0;
  @ pure
  @*/
  static int tipsAfterVision(boolean rockedBack, int tips) {
    return rockedBack && tips > 0 ? tips - 1 : satInc(tips);
  }

  // ---------------------------------------------------------------------------------------------
  // Operator corrections and restore

  /** The other CELL: every TIP, recorded, undone or seen, flips which CELL is upward. */
  /*@ normal_behavior
  @   ensures \result == !upIsA;
  @ pure
  @*/
  static boolean flipped(boolean upIsA) {
    return !upIsA;
  }

  /** TIP count after the operator records a TIP. */
  /*@ normal_behavior
  @   ensures \result == satInc(tips);
  @   ensures tips >= 0 ==> \result > tips || tips == Integer.MAX_VALUE;
  @ pure
  @*/
  static int tipsAfterRecord(int tips) {
    return satInc(tips);
  }

  /** Whether there is a TIP for the operator to undo. */
  /*@ normal_behavior
  @   ensures \result <==> tips > 0;
  @ pure
  @*/
  static boolean canUndo(int tips) {
    return tips > 0;
  }

  /** TIP count after the operator undoes a TIP: one fewer, never below zero. */
  /*@ normal_behavior
  @   ensures tips > 0 ==> \result == tips - 1;
  @   ensures tips <= 0 ==> \result == 0;
  @   ensures \result >= 0;
  @ pure
  @*/
  static int tipsAfterUndo(int tips) {
    return tips > 0 ? tips - 1 : 0;
  }

  /** A TIP count carried over from a previous OpMode, made non-negative. */
  /*@ normal_behavior
  @   ensures \result >= 0;
  @   ensures tips >= 0 ==> \result == tips;
  @ pure
  @*/
  static int restoredTips(int tips) {
    return Math.max(0, tips);
  }

  // ---------------------------------------------------------------------------------------------
  // Settling and telemetry

  /**
   * Whether the HIVE has settled: at least {@code settleMs} (a negative window counts as 0) since
   * the last change. A clock that reads earlier than the change is not settled. The {@link #NEVER}
   * sentinel is settled for any plausible clock.
   */
  /*@ normal_behavior
  @   ensures \result <==> (\bigint) nowMs - (\bigint) lastChangeMs >= (\bigint) Math.max(0L, settleMs);
  @ also normal_behavior
  @   requires nowMs >= 0 && settleMs <= Long.MAX_VALUE / 2;
  @   ensures lastChangeMs == NEVER ==> \result;
  @ pure
  @*/
  static boolean settled(long nowMs, long lastChangeMs, long settleMs) {
    return sub(nowMs, lastChangeMs) >= Math.max(0L, settleMs);
  }

  /** Milliseconds since the camera last agreed with the belief, or -1 if it never has. */
  /*@ normal_behavior
  @   ensures lastAgreeingMs == NEVER ==> \result == -1;
  @   ensures lastAgreeingMs != NEVER ==> \result == sub(nowMs, lastAgreeingMs);
  @ pure
  @*/
  static long msSinceAgreed(long nowMs, long lastAgreeingMs) {
    return lastAgreeingMs == NEVER ? -1 : sub(nowMs, lastAgreeingMs);
  }
}
