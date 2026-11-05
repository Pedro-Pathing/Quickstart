package org.firstinspires.ftc.teamcode.game;

import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Belief about this alliance's HIVE: which CELL is upward-facing, how many TIPS have happened, and
 * whether the HIVE has settled since the last one.
 *
 * <p>The HIVE is bi-stable, so its whole state is one bit plus a counter. Every match starts with
 * {@link HiveCell#A} upward (manual 10.3.1). Evidence arrives from two places:
 *
 * <ul>
 *   <li>AprilTag observations ({@link #observe}): a CELL seen above or below the upward-height
 *       threshold implies which CELL is up. A disagreement must persist for {@code confirmFrames}
 *       consecutive fresh observations before it is accepted, so one mis-ranged frame mid-swing
 *       cannot flip the state.
 *   <li>The operator ({@link #recordTip}, {@link #undoTip}), for TIPS the camera did not see.
 * </ul>
 *
 * <p>Every accepted change bumps {@link #epoch()}. Shot solutions carry the epoch they were solved
 * for, so nothing solved against the old upward CELL can authorize a feed into the new state.
 * {@link #isSettled()} stays false for {@code settleMs} after a change: the manual warns that
 * launching into a HIVE while it tips can stop the TIP, and spilled SCORING ELEMENTS are still
 * falling (G409).
 *
 * <p>Pure logic with an injected millisecond clock, so the whole state machine is unit-testable.
 */
public final class HiveTracker {

  /** Where the current belief came from, for telemetry. */
  public enum Source {
    MATCH_START,
    VISION,
    DRIVER,
    HANDOFF
  }

  private final int confirmFrames;
  private final long settleMs;
  private final long maxObservationAgeMs;
  private final LongSupplier clockMs;

  private HiveCell upCell = HiveCell.A;
  private int tips = 0;
  private int epoch = 0;
  private Source source = Source.MATCH_START;
  private long lastChangeMs = HiveLaw.NEVER;
  private long lastAgreeingObservationMs = HiveLaw.NEVER;

  private long lastObservationMs = Long.MIN_VALUE;

  private HiveCell candidateUp = HiveCell.A;
  private int candidateCount = 0;

  public HiveTracker(
      int confirmFrames, long settleMs, long maxObservationAgeMs, LongSupplier clockMs) {
    if (confirmFrames < 1) {
      throw new IllegalArgumentException("confirmFrames must be at least 1");
    }
    this.confirmFrames = confirmFrames;
    this.settleMs = Math.max(0, settleMs);
    this.maxObservationAgeMs = Math.max(0, maxObservationAgeMs);
    this.clockMs = clockMs;
  }

  public static HiveTracker fromConfig(LongSupplier clockMs) {
    var h = config.hive;
    return new HiveTracker(h.confirm_frames_count, h.settle_ms, h.max_observation_age_ms, clockMs);
  }

  /** Restores a belief carried over from a previous OpMode (AUTO to TELEOP). */
  public void restore(HiveCell upCell, int tips) {
    this.upCell = upCell;
    this.tips = HiveLaw.restoredTips(tips);
    this.epoch = HiveLaw.nextEpoch(epoch);
    this.source = Source.HANDOFF;
    this.candidateCount = 0;
  }

  /**
   * Feeds one AprilTag observation of {@code cell}.
   *
   * @param upward whether that CELL's opening was above the upward-height threshold
   * @param observedAtMs when the camera frame was captured, on the same clock as {@code clockMs}
   * @return true if this observation changed the believed state (a TIP was detected)
   */
  public boolean observe(HiveCell cell, boolean upward, long observedAtMs) {
    long now = clockMs.getAsLong();
    if (!HiveLaw.accepts(observedAtMs, lastObservationMs, now, maxObservationAgeMs)) {
      return false;
    }
    lastObservationMs = observedAtMs;
    HiveCell impliedUp = cellOf(HiveLaw.impliedUpIsA(cell == HiveCell.A, upward));
    candidateCount =
        HiveLaw.nextCandidateCount(
            impliedUp == HiveCell.A,
            upCell == HiveCell.A,
            candidateUp == HiveCell.A,
            candidateCount);
    if (impliedUp == upCell) {
      lastAgreeingObservationMs = Math.max(lastAgreeingObservationMs, observedAtMs);
      return false;
    }
    candidateUp = impliedUp;
    if (!HiveLaw.confirmed(candidateCount, confirmFrames)) {
      return false;
    }
    // A HIVE that swings past the threshold and rocks back inside the settle window is a TIP that
    // was stopped, not two TIPS: undo the first rather than counting the return as another. The
    // TIP count drives the human player's NECTAR allowance (G426), so overcounting is a foul.
    boolean rockedBack =
        HiveLaw.rockedBack(source == Source.VISION, now, lastChangeMs, settleMs, tips);
    change(impliedUp, HiveLaw.tipsAfterVision(rockedBack, tips), Source.VISION);
    lastAgreeingObservationMs = observedAtMs;
    return true;
  }

  /** Operator: a TIP just happened that the camera did not catch. */
  public void recordTip() {
    change(flippedUp(), HiveLaw.tipsAfterRecord(tips), Source.DRIVER);
  }

  /** Operator: the last TIP was a mistake. Restores the previous upward CELL. */
  public void undoTip() {
    if (HiveLaw.canUndo(tips)) {
      change(flippedUp(), HiveLaw.tipsAfterUndo(tips), Source.DRIVER);
    }
  }

  private void change(HiveCell newUp, int newTips, Source newSource) {
    upCell = newUp;
    tips = newTips;
    epoch = HiveLaw.nextEpoch(epoch);
    source = newSource;
    lastChangeMs = clockMs.getAsLong();
    candidateCount = 0;
  }

  /** The CELL believed to be upward-facing: the only legal shot target (G417). */
  public HiveCell upCell() {
    return upCell;
  }

  /** TIPS of this alliance's HIVE so far this match, including AUTO. */
  public int tips() {
    return tips;
  }

  /** Changes on every accepted state change; tags shot solutions. */
  public int epoch() {
    return epoch;
  }

  public Source source() {
    return source;
  }

  /** False while the HIVE may still be swinging or spilling after the last change. */
  public boolean isSettled() {
    return HiveLaw.settled(clockMs.getAsLong(), lastChangeMs, settleMs);
  }

  /** Milliseconds since the camera last confirmed the current belief, or -1 if it never has. */
  public long msSinceVisionAgreed() {
    return HiveLaw.msSinceAgreed(clockMs.getAsLong(), lastAgreeingObservationMs);
  }

  private static HiveCell cellOf(boolean upIsA) {
    return upIsA ? HiveCell.A : HiveCell.B;
  }

  private HiveCell flippedUp() {
    return cellOf(HiveLaw.flipped(upCell == HiveCell.A));
  }

  /** Consecutive observations currently disagreeing with the belief, for telemetry. */
  public int pendingDisagreements() {
    return candidateCount;
  }
}
