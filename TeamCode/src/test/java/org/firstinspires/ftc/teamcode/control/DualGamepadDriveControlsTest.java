package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.junit.Test;

public final class DualGamepadDriveControlsTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void mapsLeftStickToForwardAndStrafe() {
    DriveCommand command = DualGamepadDriveControls.fromInputs(0.4, -0.7, 0, 0, 1);

    assertEquals(0.7, command.getDrive(), EPSILON);
    assertEquals(0.4, command.getStrafe(), EPSILON);
    assertEquals(0.0, command.getRotate(), EPSILON);
    assertTrue(command.isActive());
  }

  @Test
  public void mapsTriggersToOppositeRotationDirections() {
    DriveCommand rotateLeft = DualGamepadDriveControls.fromInputs(0, 0, 0.8, 0, 1);
    DriveCommand rotateRight = DualGamepadDriveControls.fromInputs(0, 0, 0, 0.6, 2);

    assertEquals(-0.64, rotateLeft.getRotate(), EPSILON);
    assertEquals(0.36, rotateRight.getRotate(), EPSILON);
    assertEquals(2, rotateRight.getGamepadNumber());
  }

  @Test
  public void opposingTriggersCancelAndSmallInputsAreIgnored() {
    DriveCommand command = DualGamepadDriveControls.fromInputs(0.02, -0.03, 0.5, 0.5, 1);

    assertEquals(0.0, command.getDrive(), EPSILON);
    assertEquals(0.0, command.getStrafe(), EPSILON);
    assertEquals(0.0, command.getRotate(), EPSILON);
    assertFalse(command.isActive());
  }
}
