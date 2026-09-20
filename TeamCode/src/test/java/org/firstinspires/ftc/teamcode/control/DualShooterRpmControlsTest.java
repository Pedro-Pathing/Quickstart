package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public final class DualShooterRpmControlsTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void startsBothMotorsAtSixThousandRpm() {
    DualShooterRpmControls controls = new DualShooterRpmControls();

    assertEquals(6000.0, controls.getMotor1Rpm(), EPSILON);
    assertEquals(6000.0, controls.getMotor2Rpm(), EPSILON);
  }

  @Test
  public void adjustsEachMotorByOneHundredRpmOnButtonEdges() {
    DualShooterRpmControls controls = new DualShooterRpmControls();

    controls.updateButtons(false, false, true, true);
    assertEquals(6100.0, controls.getMotor1Rpm(), EPSILON);
    assertEquals(6100.0, controls.getMotor2Rpm(), EPSILON);

    controls.updateButtons(false, false, true, true);
    assertEquals(6100.0, controls.getMotor1Rpm(), EPSILON);
    assertEquals(6100.0, controls.getMotor2Rpm(), EPSILON);

    controls.updateButtons(false, false, false, false);
    controls.updateButtons(true, true, false, false);
    assertEquals(6000.0, controls.getMotor1Rpm(), EPSILON);
    assertEquals(6000.0, controls.getMotor2Rpm(), EPSILON);
  }
}
