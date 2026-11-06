package org.firstinspires.ftc.teamcode.subsystems.turretservo;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretServoTest {
    private static final double EPS = 0.0001;

    @Test
    public void angleToPositionUsesCenterGearRatioAndRange() {
        assertEquals(0.5, TurretServo.angleToPosition(0, 0.5, 1.25, 270, false), EPS);
        assertEquals(0.55, TurretServo.angleToPosition(0, 0.55, 1.25, 270, false), EPS);
        assertEquals(0.5 + 112.5 / 270, TurretServo.angleToPosition(Math.toRadians(90), 0.5, 1.25, 270, false), EPS);
        assertEquals(0.5 - 112.5 / 270, TurretServo.angleToPosition(Math.toRadians(-90), 0.5, 1.25, 270, false), EPS);
    }

    @Test
    public void angleToPositionReversesAroundCenter() {
        assertEquals(0.5 - 112.5 / 270, TurretServo.angleToPosition(Math.toRadians(90), 0.5, 1.25, 270, true), EPS);
    }

    @Test
    public void angleToPositionClipsToServoRange() {
        assertEquals(1, TurretServo.angleToPosition(Math.toRadians(180), 0.5, 1.25, 270, false), EPS);
        assertEquals(0, TurretServo.angleToPosition(Math.toRadians(-180), 0.5, 1.25, 270, false), EPS);
    }

    @Test
    public void estimateMovesAtMostOneStepTowardSetpoint() {
        assertEquals(0.1, TurretServo.estimateAngle(0, 1, 0.1), EPS);
        assertEquals(-0.1, TurretServo.estimateAngle(0, -1, 0.1), EPS);
        assertEquals(0.05, TurretServo.estimateAngle(0, 0.05, 0.1), EPS);
        assertEquals(1, TurretServo.estimateAngle(1, 1, 0.1), EPS);
    }
}
