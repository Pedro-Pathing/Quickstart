package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class StallTest {
    private static final double FAR = Math.toRadians(20);
    private static final double CLOSE = Math.toRadians(0.5);
    private static final double STOPPED = 0;
    private static final double MOVING = Math.toRadians(90);

    @Test
    public void pushingHardAndNotMovingIsStall() {
        assertTrue(Turret.isPushingWithoutMoving(0.8, FAR, STOPPED));
        assertTrue(Turret.isPushingWithoutMoving(-0.8, -FAR, STOPPED));
    }

    @Test
    public void movingIsNotStall() {
        assertFalse(Turret.isPushingWithoutMoving(0.8, FAR, MOVING));
    }

    @Test
    public void lowPowerIsNotStall() {
        assertFalse(Turret.isPushingWithoutMoving(0.1, FAR, STOPPED));
    }

    @Test
    public void onTargetIsNotStall() {
        assertFalse(Turret.isPushingWithoutMoving(0.8, CLOSE, STOPPED));
    }
}
