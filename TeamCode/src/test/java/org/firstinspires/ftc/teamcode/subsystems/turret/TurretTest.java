package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretTest {
    private static final double EPS = 0.0001;

    @Test
    public void ticksToAngleUsesGearRatioAndDirection() {
        assertEquals(0, Turret.ticksToAngle(0, 8192, 1, false), EPS);
        assertEquals(Math.toRadians(90), Turret.ticksToAngle(2048, 8192, 1, false), EPS);
        assertEquals(Math.toRadians(-90), Turret.ticksToAngle(2048, 8192, 1, true), EPS);
        assertEquals(Math.toRadians(45), Turret.ticksToAngle(2048, 8192, 2, false), EPS);
    }

    @Test
    public void pivotIsOffsetInRobotFrame() {
        assertArrayEquals(new double[]{10, 23}, Turret.pivot(10, 20, Math.toRadians(90), 3, 0), EPS);
        assertArrayEquals(new double[]{8, 20}, Turret.pivot(10, 20, Math.toRadians(90), 0, 2), EPS);
    }

    @Test
    public void odomAngleIsRobotRelative() {
        assertEquals(0, Turret.odomAngle(0, 0, Math.toRadians(90), 0, 50), EPS);
        assertEquals(Math.toRadians(-90), Turret.odomAngle(0, 0, 0, 0, -50), EPS);
        assertEquals(Math.toRadians(45), Turret.odomAngle(0, 0, 0, 10, 10), EPS);
    }

    @Test
    public void visionAngleRemovesRobotTurnSinceCapture() {
        assertEquals(Math.toRadians(15), Turret.visionAngle(Math.toRadians(10), Math.toRadians(5), 0, 0), EPS);
        assertEquals(Math.toRadians(5), Turret.visionAngle(Math.toRadians(10), Math.toRadians(5),
                Math.toRadians(30), Math.toRadians(20)), EPS);
    }
}
