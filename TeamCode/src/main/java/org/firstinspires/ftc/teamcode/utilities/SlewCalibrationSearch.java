package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.Nullable;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.records.SlewTuning;

/** Tests both signs on one axis; removes each unnecessary limit before refining necessary ones. */
public final class SlewCalibrationSearch {
  public enum Stage {
    BASELINE,
    FREE_INCREASE,
    INCREASE,
    FREE_DECREASE,
    DECREASE,
    COMPLETE,
    FAILED
  }

  private final double min, max, factor;
  private final int budget;
  private SlewTuning candidate;
  private @Nullable SlewTuning best;
  private Stage stage = Stage.BASELINE;
  private int count;

  public SlewCalibrationSearch(
      SlewTuning initial, double min, double max, double factor, int budget) {
    if (!Double.isFinite(min)
        || min <= 0
        || !Double.isFinite(max)
        || max < min
        || !Double.isFinite(factor)
        || factor <= 1
        || budget < 1
        || initial.longestTime() > max)
      throw new IllegalArgumentException("Invalid slew search bounds");
    this.candidate = initial;
    this.min = min;
    this.max = max;
    this.factor = factor;
    this.budget = budget;
  }

  /** Pass requires observer-confirmed contact in both signs; a witnessed lift can fail early. */
  public void record(boolean passed) {
    if (done()) throw new IllegalStateException("Search finished");
    count++;
    if (passed) best = candidate;
    if (count >= budget) {
      stage = best == null ? Stage.FAILED : Stage.COMPLETE;
      return;
    }
    switch (stage) {
      case BASELINE -> {
        if (passed) {
          if (accepted().increaseTime() == 0) tryFreeDecrease();
          else {
            stage = Stage.FREE_INCREASE;
            candidate = new SlewTuning(0, accepted().decreaseTime());
          }
        } else if (candidate.increaseTime() >= max && candidate.decreaseTime() >= max)
          stage = Stage.FAILED;
        else
          candidate =
              new SlewTuning(
                  Math.min(max, Math.max(min, candidate.increaseTime() * factor)),
                  Math.min(max, Math.max(min, candidate.decreaseTime() * factor)));
      }
      case FREE_INCREASE -> {
        if (passed) tryFreeDecrease();
        else refineIncrease();
      }
      case INCREASE -> {
        if (passed) refineIncrease();
        else tryFreeDecrease();
      }
      case FREE_DECREASE -> {
        if (passed) stage = Stage.COMPLETE;
        else refineDecrease();
      }
      case DECREASE -> {
        if (passed) refineDecrease();
        else stage = Stage.COMPLETE;
      }
      default -> throw new IllegalStateException("Search finished");
    }
  }

  /** The best passing candidate; every stage after BASELINE passed has recorded one. */
  private SlewTuning accepted() {
    return Objects.requireNonNull(best, "no passing candidate recorded yet");
  }

  private void refineIncrease() {
    SlewTuning b = accepted();
    if (b.increaseTime() <= min) {
      tryFreeDecrease();
      return;
    }
    stage = Stage.INCREASE;
    candidate = new SlewTuning(Math.max(min, b.increaseTime() / factor), b.decreaseTime());
  }

  private void tryFreeDecrease() {
    SlewTuning b = accepted();
    if (b.decreaseTime() == 0) {
      stage = Stage.COMPLETE;
      return;
    }
    stage = Stage.FREE_DECREASE;
    candidate = new SlewTuning(b.increaseTime(), 0);
  }

  private void refineDecrease() {
    SlewTuning b = accepted();
    if (b.decreaseTime() <= min) {
      stage = Stage.COMPLETE;
      return;
    }
    stage = Stage.DECREASE;
    candidate = new SlewTuning(b.increaseTime(), Math.max(min, b.decreaseTime() / factor));
  }

  public SlewTuning candidate() {
    return candidate;
  }

  public @Nullable SlewTuning best() {
    return best;
  }

  public Stage stage() {
    return stage;
  }

  public int candidates() {
    return count;
  }

  public boolean done() {
    return stage == Stage.COMPLETE || stage == Stage.FAILED;
  }
}
