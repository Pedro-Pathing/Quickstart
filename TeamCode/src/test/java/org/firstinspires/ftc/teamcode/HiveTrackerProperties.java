package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.constraints.IntRange;
import net.jqwik.api.constraints.LongRange;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.HiveCell;

/**
 * Drives {@link HiveTracker} through random sequences of camera observations, operator TIP
 * corrections and clock advances, and checks after every step the guarantees the shot gate relies
 * on: the epoch changes exactly when the believed state does, the upward CELL is always consistent
 * with the TIP count, stale or late frames never flip the HIVE, one mis-ranged frame cannot flip it
 * either, and the settle window is honoured. Failing sequences shrink to a minimal reproduction.
 */
class HiveTrackerProperties {

  sealed interface Op permits Advance, Observe, RecordTip, UndoTip {}

  record Advance(long ms) implements Op {}

  /** A camera frame of {@code cell} captured {@code ageMs} before it is delivered. */
  record Observe(HiveCell cell, boolean upward, long ageMs) implements Op {}

  record RecordTip() implements Op {}

  record UndoTip() implements Op {}

  record Params(int confirmFrames, long settleMs, long maxAgeMs) {}

  @Property(tries = 500)
  void everyStepPreservesTheTrackerInvariants(
      @ForAll("params") Params p, @ForAll("operations") List<Op> ops) {
    long[] now = {10_000};
    HiveTracker tracker =
        new HiveTracker(p.confirmFrames(), p.settleMs(), p.maxAgeMs(), () -> now[0]);

    long lastChangeAt = Long.MIN_VALUE;
    long newestAcceptedFrame = Long.MIN_VALUE;
    int observationsSinceChange = 0;

    for (Op op : ops) {
      HiveCell upBefore = tracker.upCell();
      int tipsBefore = tracker.tips();
      int epochBefore = tracker.epoch();
      boolean reported = false;

      switch (op) {
        case Advance a -> now[0] += a.ms();
        case Observe o -> {
          long capturedAt = now[0] - o.ageMs();
          boolean fresh = capturedAt > newestAcceptedFrame && o.ageMs() <= p.maxAgeMs();
          if (fresh) {
            newestAcceptedFrame = capturedAt;
            observationsSinceChange++;
          }
          reported = tracker.observe(o.cell(), o.upward(), capturedAt);

          if (!fresh) {
            assertFalse(reported, "a stale or out-of-order frame changed the HIVE: " + o);
          }
          if (reported) {
            HiveCell implied = o.upward() ? o.cell() : o.cell().other();
            assertEquals(implied, tracker.upCell(), "flipped against the evidence");
            assertTrue(
                observationsSinceChange >= p.confirmFrames(),
                "flipped after "
                    + observationsSinceChange
                    + " fresh frames; needs "
                    + p.confirmFrames());
          }
        }
        case RecordTip r -> tracker.recordTip();
        case UndoTip u -> tracker.undoTip();
      }

      boolean changed = tracker.upCell() != upBefore;
      if (op instanceof Observe) {
        assertEquals(changed, reported, "observe() return value disagrees with the state");
      }
      if (op instanceof RecordTip) {
        assertTrue(changed, "recordTip() must always flip the HIVE");
      }
      if (op instanceof UndoTip) {
        assertEquals(tipsBefore > 0, changed, "undoTip() flips only when there is a TIP to undo");
      }

      // The shot gate trusts the epoch: it must move on every change and only then.
      assertEquals(changed ? epochBefore + 1 : epochBefore, tracker.epoch());
      assertEquals(changed ? 1 : 0, Math.abs(tracker.tips() - tipsBefore));
      // Every match starts CELL A up and every change is one TIP, so parity pins the upward CELL.
      assertTrue(tracker.tips() >= 0);
      assertEquals(tracker.tips() % 2 == 0 ? HiveCell.A : HiveCell.B, tracker.upCell());

      if (changed) {
        lastChangeAt = now[0];
        observationsSinceChange = 0;
      }
      boolean shouldBeSettled =
          lastChangeAt == Long.MIN_VALUE || now[0] - lastChangeAt >= p.settleMs();
      assertEquals(shouldBeSettled, tracker.isSettled(), "settle window");
    }
  }

  /**
   * Liveness: from any reachable state, {@code confirmFrames} consistent fresh frames do flip it.
   */
  @Property(tries = 300)
  void enoughConsistentFreshFramesAlwaysFlip(
      @ForAll("params") Params p,
      @ForAll("operations") List<Op> history,
      @ForAll @LongRange(min = 0, max = 50) long frameSpacingMs) {
    long[] now = {10_000};
    HiveTracker tracker =
        new HiveTracker(p.confirmFrames(), p.settleMs(), p.maxAgeMs(), () -> now[0]);
    for (Op op : history) {
      switch (op) {
        case Advance a -> now[0] += a.ms();
        case Observe o -> tracker.observe(o.cell(), o.upward(), now[0] - o.ageMs());
        case RecordTip r -> tracker.recordTip();
        case UndoTip u -> tracker.undoTip();
      }
    }

    HiveCell wasUp = tracker.upCell();
    int epoch = tracker.epoch();
    int flips = 0;
    for (int i = 0; i < p.confirmFrames(); i++) {
      now[0] += frameSpacingMs + 1;
      // The camera now sees the formerly upward CELL hanging low.
      if (tracker.observe(wasUp, false, now[0])) {
        flips++;
      }
    }

    assertEquals(1, flips);
    assertEquals(wasUp.other(), tracker.upCell());
    assertEquals(epoch + 1, tracker.epoch());
  }

  @Property
  void constructorRejectsANonPositiveConfirmCount(@ForAll @IntRange(min = -5, max = 0) int frames) {
    org.junit.jupiter.api.Assertions.assertThrows(
        IllegalArgumentException.class, () -> new HiveTracker(frames, 0, 0, () -> 0L));
  }

  @Provide
  Arbitrary<Params> params() {
    return Combinators.combine(
            Arbitraries.integers().between(1, 5),
            Arbitraries.longs().between(0, 1_000),
            Arbitraries.longs().between(0, 500))
        .as(Params::new);
  }

  @Provide
  Arbitrary<List<Op>> operations() {
    Arbitrary<Op> advance = Arbitraries.longs().between(0, 400).map(Advance::new);
    Arbitrary<Op> observe =
        Combinators.combine(
                Arbitraries.of(HiveCell.class),
                Arbitraries.of(true, false),
                Arbitraries.longs().between(0, 700))
            .as(Observe::new);
    Arbitrary<Op> recordTip = Arbitraries.just(new RecordTip());
    Arbitrary<Op> undoTip = Arbitraries.just(new UndoTip());
    // Weighted toward observations: the debounce is the subtle part.
    return Arbitraries.frequencyOf(
            net.jqwik.api.Tuple.of(3, advance),
            net.jqwik.api.Tuple.of(6, observe),
            net.jqwik.api.Tuple.of(1, recordTip),
            net.jqwik.api.Tuple.of(1, undoTip))
        .list()
        .ofMaxSize(80);
  }
}
