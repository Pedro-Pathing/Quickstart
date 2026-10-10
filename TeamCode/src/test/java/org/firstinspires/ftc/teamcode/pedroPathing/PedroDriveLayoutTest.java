package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.assertArrayEquals;
import org.junit.Test;

public final class PedroDriveLayoutTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void forwardMatchesTheConfirmedInstalledPattern() {
    assertArrayEquals(new double[] {-1, 1, 1, -1},
        PedroDriveLayout.remap(new double[] {1, 1, 1, 1}, true, true), EPSILON);
  }

  @Test
  public void leftStrafeMatchesTheConfirmedInstalledPattern() {
    assertArrayEquals(new double[] {1, 1, 1, 1},
        PedroDriveLayout.remap(new double[] {-1, 1, 1, -1}, true, true), EPSILON);
  }

  @Test
  public void counterclockwiseTurnMatchesLeftTriggerPattern() {
    assertArrayEquals(new double[] {1, 1, -1, -1},
        PedroDriveLayout.remap(new double[] {-1, -1, 1, 1}, true, true), EPSILON);
  }

  @Test
  public void standardLayoutRetainsPedroWheelOrderAndPowers() {
    double[] powers = {0.2, 0.4, -0.1, -0.3};
    assertArrayEquals(powers, PedroDriveLayout.remap(powers, false, false), EPSILON);
  }

  @Test
  public void mixedMotionRetainsPowerScaleAndStopsExactly() {
    assertArrayEquals(new double[] {-0.2, -0.1, 0.4, 0.3},
        PedroDriveLayout.remap(new double[] {0.2, 0.4, -0.1, -0.3}, true, true), EPSILON);
    assertArrayEquals(new double[4], PedroDriveLayout.remap(new double[4], true, true), EPSILON);
  }
}
