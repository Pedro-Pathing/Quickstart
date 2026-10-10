package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.*;
import com.pedropathing.geometry.Pose;
import java.util.Collections;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.junit.Test;

public final class PedroAprilTagValidationTest {
  private static final double EPSILON = 1e-9;

  private static AprilTagObservation tag(int id, double forward, double left) {
    return new AprilTagObservation(id, forward, left, 8, 0, 0);
  }

  @Test
  public void convertsPedroRadiansAndConfirmsTranslationFromBaselineTag() {
    PedroAprilTagValidation check = new PedroAprilTagValidation(1, 1, true);
    check.update(new Pose(12, -2, Math.PI / 2), Collections.singletonList(tag(5, 20, 4)),
        true, false, false);
    check.update(new Pose(12, 3, Math.PI / 2), Collections.singletonList(tag(5, 15, 4)),
        true, false, false);
    assertTrue(check.getResult().isPassing());
    assertEquals(12, check.getVisionPose().getX(), EPSILON);
    assertEquals(3, check.getVisionPose().getY(), EPSILON);
    assertEquals(90, check.getVisionPose().getHeadingDegrees(), EPSILON);
  }

  @Test
  public void detectsDriftWithoutReanchoringOrChangingInputPose() {
    PedroAprilTagValidation check = new PedroAprilTagValidation(1, 1, true);
    check.update(new Pose(0, 0, 0), Collections.singletonList(tag(5, 24, 0)), true, false, false);
    Pose pedro = new Pose(6, 0, 0);
    check.update(pedro, Collections.singletonList(tag(5, 14, 0)), true, false, false);
    assertFalse(check.getResult().isPassing());
    assertEquals(4, check.getResult().getPositionErrorInches(), EPSILON);
    assertEquals(10, check.getVisionPose().getX(), EPSILON);
    assertEquals(6, pedro.getX(), EPSILON);
  }

  @Test
  public void missingDifferentOrDisconnectedTagCannotPassAndDoesNotReplaceBaseline() {
    PedroAprilTagValidation check = new PedroAprilTagValidation(1, 1, true);
    check.update(new Pose(), Collections.singletonList(tag(5, 24, 0)), true, false, false);
    check.update(new Pose(), Collections.singletonList(tag(9, 24, 0)), true, false, false);
    assertFalse(check.getResult().isAvailable());
    assertEquals(Integer.valueOf(5), check.getTrackedTagId());
    check.update(new Pose(), Collections.singletonList(tag(5, 24, 0)), false, false, false);
    assertFalse(check.getResult().isAvailable());
    assertNull(check.getVisionPose());
    check.update(new Pose(), Collections.singletonList(tag(5, 24, 0)), true, false, false);
    assertTrue(check.getResult().isPassing());
  }

  @Test
  public void clearDisablesAutomaticCaptureAndACapturesOnlyOnPress() {
    PedroAprilTagValidation check = new PedroAprilTagValidation(1, 1, true);
    check.update(new Pose(), Collections.singletonList(tag(5, 24, 0)), true, false, false);
    check.update(new Pose(), Collections.singletonList(tag(5, 24, 0)), true, false, true);
    check.update(new Pose(), Collections.singletonList(tag(5, 24, 0)), true, false, false);
    assertNull(check.getTrackedTagId());
    check.update(new Pose(), Collections.singletonList(tag(9, 24, 0)), true, true, false);
    assertEquals(Integer.valueOf(9), check.getTrackedTagId());
    check.update(new Pose(), Collections.singletonList(tag(5, 24, 0)), true, true, false);
    assertEquals(Integer.valueOf(9), check.getTrackedTagId());
  }

  @Test
  public void diagnosticRequiresManualBaselineAndHandlesMissingTags() {
    PedroAprilTagValidation check = new PedroAprilTagValidation(1, 1, false);
    check.update(new Pose(), Collections.singletonList(tag(5, 24, 0)), true, false, false);
    assertNull(check.getTrackedTagId());
    assertFalse(check.getResult().isAvailable());
    check.update(new Pose(), Collections.emptyList(), true, true, false);
    assertNull(check.getTrackedTagId());
  }
}
