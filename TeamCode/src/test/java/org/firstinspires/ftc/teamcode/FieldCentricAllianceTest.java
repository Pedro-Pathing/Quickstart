package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * Field-centric stick-forward drives away from this alliance's drivers. RED drivers stand at the x
 * = 0 wall and BLUE drivers at the far wall, so the same stick must mean +x for RED and -x for
 * BLUE; a single shared offset made every BLUE axis inverted.
 */
public class FieldCentricAllianceTest {
  private final boolean friction = Casablanca.enableFrictionComp;
  private final boolean smoothing = Casablanca.enableInputSmoothing;
  private final boolean headingLock = Casablanca.enableHeadingLock;
  private final boolean fieldCentric = Casablanca.fieldCentric;
  private final double offset = Casablanca.fieldCentricOffsetRad;

  @AfterEach
  public void restore() {
    Casablanca.enableFrictionComp = friction;
    Casablanca.enableInputSmoothing = smoothing;
    Casablanca.enableHeadingLock = headingLock;
    Casablanca.fieldCentric = fieldCentric;
    Casablanca.fieldCentricOffsetRad = offset;
  }

  private static double[] fullForward(Alliance alliance, double headingRad) {
    Casablanca c = new Casablanca(new Sentinel(alliance));
    Casablanca.enableFrictionComp = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableHeadingLock = false;
    Casablanca.fieldCentric = true;
    Casablanca.fieldCentricOffsetRad = 0.0;
    // Mid-field corner, far from the HIVE, so protection does not touch the command.
    Pose pose = new Pose(20, 20, headingRad);
    return c.adjustDriveInput(pose, Vector2D.zero(), 0, 0.0, 1.0, 0.0);
  }

  @Test
  public void stickForwardDrivesAwayFromEachAlliancesDrivers() {
    for (double heading : new double[] {0.0, Math.PI / 2, -2.0}) {
      double[] red = fullForward(Alliance.RED, heading);
      assertEquals(1.0, red[1], 1e-9, "RED forward is +x at heading " + heading);
      assertEquals(0.0, red[0], 1e-9);

      double[] blue = fullForward(Alliance.BLUE, heading);
      assertEquals(-1.0, blue[1], 1e-9, "BLUE forward is -x at heading " + heading);
      assertEquals(0.0, blue[0], 1e-9);
    }
  }
}
