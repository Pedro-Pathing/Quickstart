package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.junit.Test;

public class TurretHistoryTest {
    private static final double EPS = 0.0001;

    @Test
    public void emptyHistoryHasNoSample() {
        assertNull(new TurretHistory().at(0));
    }

    @Test
    public void interpolatesBetweenSamples() {
        TurretHistory h = new TurretHistory();
        h.add(1.0, 0.0, 0, 0, 0);
        h.add(2.0, 1.0, 10, 20, 0.5);
        TurretHistory.Sample s = h.at(1.25);
        assertEquals(0.25, s.turretAngle, EPS);
        assertEquals(2.5, s.x, EPS);
        assertEquals(5, s.y, EPS);
        assertEquals(0.125, s.heading, EPS);
    }

    @Test
    public void headingInterpolatesAcrossPlusMinusPi() {
        TurretHistory h = new TurretHistory();
        h.add(0.0, 0, 0, 0, Math.PI - 0.1);
        h.add(0.1, 0, 0, 0, -Math.PI + 0.1);
        assertEquals(Math.PI, Math.abs(Turret.wrap(h.at(0.05).heading)), EPS);
    }

    @Test
    public void outsideTheBufferUsesTheEnds() {
        TurretHistory h = new TurretHistory();
        h.add(1.0, 1, 0, 0, 0);
        h.add(2.0, 2, 0, 0, 0);
        assertEquals(1, h.at(0).turretAngle, EPS);
        assertEquals(2, h.at(5.0).turretAngle, EPS);
    }

    @Test
    public void oldSamplesAreOverwritten() {
        TurretHistory h = new TurretHistory();
        for (int i = 0; i < TurretHistory.SIZE + 10; i++) h.add(i, i, 0, 0, 0);

        assertEquals(10, h.at(0).turretAngle, EPS);
        assertEquals(40, h.at(40).turretAngle, EPS);
    }
}
