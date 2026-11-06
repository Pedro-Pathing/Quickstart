package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HiveTargetTest {
    private static HiveTarget at(double x, double y) {
        return new HiveTarget(HiveTarget.Cell.SCORING, x, y, 1.0, 2, 1, 0, 1, 0);
    }

    @Test
    public void straightAheadKeepsHeading() {
        assertEquals(0.4, at(1.0, 0).targetHeading(0.4), 1e-9);
    }

    @Test
    public void targetToTheLeftTurnsCounterClockwise() {
        assertEquals(Math.PI / 4, at(1.0, 1.0).targetHeading(0), 1e-9);
        assertEquals(Math.PI / 4 + 0.5, at(1.0, 1.0).targetHeading(0.5), 1e-9);
    }

    @Test
    public void targetToTheRightTurnsClockwise() {
        assertEquals(-Math.PI / 4, at(1.0, -1.0).targetHeading(0), 1e-9);
    }

    @Test
    public void resultIsNormalised() {
        double h = at(-1.0, 0.0).targetHeading(0.5);
        assertTrue(h > -Math.PI && h <= Math.PI);
        assertEquals(Math.PI + 0.5 - 2 * Math.PI, h, 1e-9);
    }

    @Test
    public void zeroOffsetMatchesBearing() {
        HiveTarget t = at(1.5, 0.4);
        assertEquals(t.targetHeading(0.2), t.targetHeading(0.2, 0.0), 1e-12);
        assertEquals(Math.atan2(0.4, 1.5), t.bearing(), 1e-12);
    }

    @Test
    public void launcherOnTheLeftAimsTheRobotFurtherRight() {
        assertEquals(-Math.asin(0.2 / 2.0), at(2.0, 0).targetHeading(0, 0.2), 1e-9);
    }

    @Test
    public void launcherOnTheRightAimsTheRobotFurtherLeft() {
        assertEquals(Math.asin(0.15 / 2.0), at(2.0, 0).targetHeading(0, -0.15), 1e-9);
    }

    @Test
    public void offsetLauncherLinePassesThroughTarget() {
        HiveTarget t = at(1.8, 0.5);
        double offset = 0.12;
        double heading = t.targetHeading(0, offset);
        // Launcher line starts offset to the robot's left and runs along the new heading, with the
        // robot's centre at the origin. The target's sideways distance from that line should be zero
        double lx = -Math.sin(heading) * offset, ly = Math.cos(heading) * offset;
        double side = -(t.x - lx) * Math.sin(heading) + (t.y - ly) * Math.cos(heading);
        assertEquals(0.0, side, 1e-9);
    }

    @Test
    public void offsetLargerThanRangeIsClamped() {
        assertEquals(-Math.PI / 2, at(0.1, 0).targetHeading(0, 5.0), 1e-9);
    }

    @Test
    public void distances() {
        HiveTarget t = at(3, 4);
        assertEquals(5, t.horizontalDistance(), 1e-12);
        assertEquals(Math.hypot(2, 5), t.horizontalDistanceFrom(1, -1), 1e-12);
    }

    @Test
    public void targetOnTheRobotKeepsHeading() {
        assertEquals(0.7, at(0, 0).targetHeading(0.7), 1e-12);
    }
}
