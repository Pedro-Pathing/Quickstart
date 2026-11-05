package org.firstinspires.ftc.teamcode.game;

import androidx.annotation.Nullable;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Time-dependent BIOBUZZ rules for one OpMode period (AUTO or TELEOP).
 *
 * <p>Robot code cannot see the field timer, so it counts from {@link #start()}, which the OpMode
 * calls when the drivers press play. The cues it raises are reminders, not enforcement: the
 * official FIELD timer remains the authority (manual 9.11).
 */
public final class MatchClock {

  public enum Period {
    AUTO,
    TELEOP
  }

  /** One-shot reminders raised as the period crosses a rule boundary. */
  public enum Cue {
    /** G410 / G426: NECTAR may now enter FLOWERS; humans may enter all remaining NECTAR. */
    FLOWER_UNLOCK
  }

  /** NECTAR initially staged in each ALLIANCE AREA (manual 10.3.1.B). */
  public static final int STAGED_NECTAR = 5;

  private final Period period;
  private final long periodMs;
  private final long flowerUnlockRemainingMs;
  private final long finalWarningRemainingMs;
  private final LongSupplier clockMs;

  private long startMs = Long.MIN_VALUE;
  private boolean flowerCueRaised = false;

  public MatchClock(
      Period period,
      long periodMs,
      long flowerUnlockRemainingMs,
      long finalWarningRemainingMs,
      LongSupplier clockMs) {
    this.period = period;
    this.periodMs = periodMs;
    this.flowerUnlockRemainingMs = flowerUnlockRemainingMs;
    this.finalWarningRemainingMs = finalWarningRemainingMs;
    this.clockMs = clockMs;
  }

  public static MatchClock fromConfig(Period period, LongSupplier clockMs) {
    var m = config.match;
    long length = period == Period.AUTO ? m.auto_ms : m.teleop_ms;
    return new MatchClock(
        period, length, m.flower_unlock_remaining_ms, m.final_warning_remaining_ms, clockMs);
  }

  /** Marks the moment the drivers pressed play. */
  public void start() {
    startMs = clockMs.getAsLong();
    flowerCueRaised = false;
  }

  public boolean isStarted() {
    return startMs != Long.MIN_VALUE;
  }

  public Period period() {
    return period;
  }

  public long elapsedMs() {
    return isStarted() ? Math.max(0, clockMs.getAsLong() - startMs) : 0;
  }

  /** Time left in this period, never negative. The full period before {@link #start()}. */
  public long remainingMs() {
    return Math.max(0, periodMs - elapsedMs());
  }

  /**
   * G410 / G426: whether NECTAR may enter FLOWER scoring volumes and humans may enter every
   * remaining NECTAR. Only ever true in TELEOP's final minute.
   */
  public boolean isFlowerUnlocked() {
    return period == Period.TELEOP && isStarted() && remainingMs() <= flowerUnlockRemainingMs;
  }

  /**
   * The window, from {@code finalWarningRemainingMs} to the buzzer, in which parking is watched.
   */
  public boolean isFinalWarning() {
    return isStarted() && remainingMs() <= finalWarningRemainingMs;
  }

  /**
   * The park alert: inside the final-warning window, the time left is no more than the estimated
   * drive to the park pose plus {@code marginMs}. A drive that cannot be estimated ({@code
   * driveSeconds} infinite or NaN, e.g. no clear route) alerts as soon as the window opens.
   */
  public boolean isParkAlert(double driveSeconds, long marginMs) {
    if (!isFinalWarning()) {
      return false;
    }
    if (!(driveSeconds >= 0) || Double.isInfinite(driveSeconds)) {
      return true;
    }
    return remainingMs() <= driveSeconds * 1000.0 + marginMs;
  }

  /**
   * AUTO's park deadline: time to stop the routine and drive to park. Due once the time left is no
   * more than the estimated drive ({@code driveSeconds}, including stopping from the current speed)
   * plus {@code marginMs}, and never later than {@code minReserveMs} before the end. A drive that
   * cannot be estimated (infinite or NaN) falls back to {@code minReserveMs} alone, so a moment
   * with no clear route never cancels the routine early.
   */
  public boolean isParkDue(double driveSeconds, long marginMs, long minReserveMs) {
    if (!isStarted()) {
      return false;
    }
    double deadlineMs = minReserveMs;
    if (driveSeconds >= 0 && Double.isFinite(driveSeconds)) {
      deadlineMs = Math.max(deadlineMs, driveSeconds * 1000.0 + marginMs);
    }
    return remainingMs() <= deadlineMs;
  }

  /** Returns each cue once, the first time it is polled after its boundary is crossed. */
  public @Nullable Cue pollCue() {
    if (!flowerCueRaised && isFlowerUnlocked()) {
      flowerCueRaised = true;
      return Cue.FLOWER_UNLOCK;
    }
    return null;
  }

  /**
   * NECTAR this alliance's humans may have entered so far (G426): one per TIP of their HIVE, or all
   * of them once the final minute starts. Humans subtract what they have already entered.
   */
  public static int nectarAllowance(int ownTips, boolean flowerUnlocked) {
    if (flowerUnlocked) {
      return STAGED_NECTAR;
    }
    return Math.clamp(ownTips, 0, STAGED_NECTAR);
  }
}
