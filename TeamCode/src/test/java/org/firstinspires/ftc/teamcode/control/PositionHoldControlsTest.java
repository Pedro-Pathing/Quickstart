package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.control.PositionHoldControls.Mode;
import org.firstinspires.ftc.teamcode.control.PositionHoldControls.Transition;
import org.junit.Test;

public final class PositionHoldControlsTest {
  private static final double EPSILON = 1e-9;

  private static PositionHoldControls create() {
    return new PositionHoldControls(2.0, Math.toRadians(10), 0.5);
  }

  @Test
  public void locksPoseOnceRobotStopsAfterDriverReleases() {
    PositionHoldControls hold = create();
    assertEquals(Transition.NONE, hold.update(true, 0, 0, 0, 0.00));
    assertEquals(Transition.NONE, hold.update(true, 1, 0, 0, 0.02));
    // Released while still coasting at 25 in/s: keep braking.
    assertEquals(Transition.NONE, hold.update(false, 1.5, 0, 0, 0.04));
    assertEquals(Mode.SETTLING, hold.getMode());
    // Nearly stopped: lock the current pose.
    assertEquals(Transition.START_HOLD, hold.update(false, 1.52, 0, 0.001, 0.06));
    assertEquals(Mode.HOLDING, hold.getMode());
    assertEquals(1.52, hold.getHoldX(), EPSILON);
    assertEquals(0.001, hold.getHoldHeading(), EPSILON);
  }

  @Test
  public void pushWhileHoldingKeepsOriginalTarget() {
    PositionHoldControls hold = create();
    hold.update(false, 5, 5, 0, 0.00);
    assertEquals(Transition.START_HOLD, hold.update(false, 5, 5, 0, 0.02));
    // A fast push does not move the target or leave hold.
    assertEquals(Transition.NONE, hold.update(false, 8, 1, 0.5, 0.04));
    assertEquals(Mode.HOLDING, hold.getMode());
    assertEquals(5, hold.getHoldX(), EPSILON);
    assertEquals(5, hold.getHoldY(), EPSILON);
    assertEquals(5, hold.distanceFromHold(8, 1), EPSILON);
    assertEquals(-0.5, hold.headingErrorFromHold(0.5), EPSILON);
  }

  @Test
  public void driverInputLeavesHoldAndLaterLocksNewPose() {
    PositionHoldControls hold = create();
    hold.update(false, 0, 0, 0, 0.00);
    hold.update(false, 0, 0, 0, 0.02);
    assertEquals(Transition.START_DRIVER, hold.update(true, 0, 0, 0, 0.04));
    assertEquals(Transition.NONE, hold.update(true, 10, 0, 0, 0.5));
    assertEquals(Mode.DRIVER, hold.getMode());
    // Already stationary when released, so the new pose locks on the same loop.
    assertEquals(Transition.START_HOLD, hold.update(false, 10, 0, 0, 0.52));
    assertEquals(10, hold.getHoldX(), EPSILON);
  }

  @Test
  public void locksAfterTimeoutEvenIfStillMoving() {
    PositionHoldControls hold = create();
    hold.update(true, 0, 0, 0, 0.0);
    assertEquals(Transition.NONE, hold.update(false, 1, 0, 0, 0.1));
    assertEquals(Transition.NONE, hold.update(false, 2, 0, 0, 0.3));
    assertEquals(Transition.START_HOLD, hold.update(false, 3, 0, 0, 0.6));
    assertEquals(3, hold.getHoldX(), EPSILON);
  }

  @Test
  public void spinningInPlaceIsNotTreatedAsStopped() {
    PositionHoldControls hold = create();
    hold.update(true, 0, 0, 0, 0.0);
    assertEquals(Transition.NONE, hold.update(false, 0, 0, 0.1, 0.02));
    assertEquals(Mode.SETTLING, hold.getMode());
  }
}
