package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ShotTableTest {
    @Test
    public void printsPairsSortedByDistance() {
        ShotTable table = new ShotTable(3);
        table.record(72, 1400);
        table.record(24.04, 1000);
        table.record(48, 1200);
        assertEquals("SHOOTER_DIST_IN = {24.0, 48.0, 72.0};", table.distanceLine());
        assertEquals("SHOOTER_RPM = {1000, 1200, 1400};", table.velocityLine());
    }

    @Test
    public void nearbyDistanceReplacesOldShot() {
        ShotTable table = new ShotTable(3);
        table.record(48, 1200);
        table.record(50, 1250);
        assertEquals(1, table.size());
        assertEquals("SHOOTER_RPM = {1250};", table.velocityLine());
    }

    @Test
    public void clearRemovesEveryRecord() {
        ShotTable table = new ShotTable(3);
        table.record(96, 1600);
        table.record(24, 1000);
        table.clear();
        assertEquals(0, table.size());
        assertEquals("SHOOTER_DIST_IN = {};", table.distanceLine());
    }

    @Test
    public void unknownDistanceIsIgnored() {
        ShotTable table = new ShotTable(3);
        table.record(Double.NaN, 1000);
        assertEquals(0, table.size());
    }
}
