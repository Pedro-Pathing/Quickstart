package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.*;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.OdometryMovementSequence.Step;
import org.junit.Test;

public final class OdometryMovementSequenceTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void fourSeparatePressesRunRequestedDirectionsAndRepeat() {
    OdometryMovementSequence sequence = new OdometryMovementSequence();
    Step[] expected = {Step.FORWARD, Step.LEFT, Step.BACKWARD, Step.RIGHT, Step.FORWARD};
    for (Step step : expected) {
      sequence.updateY(false, true);
      assertEquals(step, sequence.updateY(true, true));
      assertTrue(sequence.isRunning());
      sequence.complete();
      assertFalse(sequence.isRunning());
    }
  }

  @Test
  public void holdingYAfterCompletionCannotStartAnotherLeg() {
    OdometryMovementSequence sequence = new OdometryMovementSequence();
    assertEquals(Step.FORWARD, sequence.updateY(true, true));
    sequence.complete();
    assertNull(sequence.updateY(true, true));
    assertFalse(sequence.isRunning());
    sequence.updateY(false, true);
    assertEquals(Step.LEFT, sequence.updateY(true, true));
  }

  @Test
  public void pressesDuringMovementAreNotQueued() {
    OdometryMovementSequence sequence = new OdometryMovementSequence();
    sequence.updateY(true, true);
    sequence.updateY(false, true);
    assertNull(sequence.updateY(true, true));
    sequence.complete();
    assertNull(sequence.updateY(true, true));
    assertEquals(Step.LEFT, sequence.getNextStep());
  }

  @Test
  public void disabledPressMustBeReleasedBeforeMovementCanStart() {
    OdometryMovementSequence sequence = new OdometryMovementSequence();
    assertNull(sequence.updateY(true, false));
    assertNull(sequence.updateY(true, true));
    sequence.updateY(false, true);
    assertEquals(Step.FORWARD, sequence.updateY(true, true));
  }

  @Test
  public void abortRetriesSameDirectionWithoutAdvancing() {
    OdometryMovementSequence sequence = new OdometryMovementSequence();
    sequence.updateY(true, true);
    sequence.abort();
    assertEquals(Step.FORWARD, sequence.getNextStep());
    assertFalse(sequence.isRunning());
    sequence.updateY(false, true);
    assertEquals(Step.FORWARD, sequence.updateY(true, true));
  }

  @Test
  public void fourTwelveInchLegsReturnToOriginWithoutTurning() {
    Pose start = new Pose(72, 72, 0);
    Pose pose = start;
    double[][] targets = {{84, 72}, {84, 84}, {72, 84}, {72, 72}};
    int index = 0;
    for (Step step : Step.values()) {
      Pose next = step.targetFrom(pose);
      assertEquals(12, Math.hypot(next.getX() - pose.getX(), next.getY() - pose.getY()), EPSILON);
      assertEquals(targets[index][0], next.getX(), EPSILON);
      assertEquals(targets[index][1], next.getY(), EPSILON);
      assertEquals(0, next.getHeading(), EPSILON);
      pose = next;
      index++;
    }
  }

  @Test
  public void targetsAreRobotRelativeAtRotatedHeadings() {
    Pose start = new Pose(5, -3, Math.PI / 2);
    Pose forward = Step.FORWARD.targetFrom(start);
    Pose left = Step.LEFT.targetFrom(start);
    assertEquals(5, forward.getX(), EPSILON);
    assertEquals(9, forward.getY(), EPSILON);
    assertEquals(-7, left.getX(), EPSILON);
    assertEquals(-3, left.getY(), EPSILON);
    assertEquals(start.getHeading(), forward.getHeading(), EPSILON);
    Pose pose = start;
    for (Step step : Step.values()) {
      pose = step.targetFrom(pose);
    }
    assertEquals(start.getX(), pose.getX(), EPSILON);
    assertEquals(start.getY(), pose.getY(), EPSILON);
  }
}
