package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class PoseHistoryTest {
    private static final long MS = 1_000_000L;

    @Test
    public void emptyHistoryGivesNull() {
        assertNull(new PoseHistory().at(0));
    }

    @Test
    public void interpolatesBetweenPoses() {
        PoseHistory h = new PoseHistory();
        h.record(0, new Pose(0, 0, 0));
        h.record(100 * MS, new Pose(10, 20, 1.0));
        Pose p = h.at(25 * MS);
        assertEquals(2.5, p.x(), 1e-9);
        assertEquals(5.0, p.y(), 1e-9);
        assertEquals(0.25, p.heading(), 1e-9);
    }

    @Test
    public void interpolatesHeadingAcrossPlusMinusPi() {
        PoseHistory h = new PoseHistory();
        h.record(0, new Pose(0, 0, Math.PI - 0.1));
        h.record(100 * MS, new Pose(0, 0, -Math.PI + 0.1));
        // Halfway is at pi, the short way round, not at 0
        assertEquals(Math.PI, Math.abs(h.at(50 * MS).heading()), 1e-9);
        // A quarter of the way is 0.05 further on from the first heading
        assertEquals(Math.PI - 0.05, h.at(25 * MS).heading(), 1e-9);
    }

    @Test
    public void beforeOldestGivesOldest() {
        PoseHistory h = new PoseHistory();
        h.record(100 * MS, new Pose(1, 2, 0.3));
        h.record(200 * MS, new Pose(5, 6, 0.4));
        Pose p = h.at(0);
        assertEquals(1, p.x(), 1e-9);
        assertEquals(2, p.y(), 1e-9);
    }

    @Test
    public void afterNewestGivesNewest() {
        PoseHistory h = new PoseHistory();
        h.record(100 * MS, new Pose(1, 2, 0.3));
        h.record(200 * MS, new Pose(5, 6, 0.4));
        Pose p = h.at(900 * MS);
        assertEquals(5, p.x(), 1e-9);
        assertEquals(6, p.y(), 1e-9);
    }

    @Test
    public void exactTimestampGivesThatPose() {
        PoseHistory h = new PoseHistory();
        h.record(0, new Pose(1, 1, 0));
        h.record(100 * MS, new Pose(2, 2, 0));
        h.record(200 * MS, new Pose(3, 3, 0));
        assertEquals(2, h.at(100 * MS).x(), 1e-9);
    }

    @Test
    public void posesOlderThanKeepWindowAreNotUsed() {
        PoseHistory h = new PoseHistory();
        h.record(0, new Pose(0, 0, 0));
        h.record(100 * MS, new Pose(100, 0, 0));
        h.record(700 * MS, new Pose(700, 0, 0));
        // Asking for a time before the 500 ms window gets the oldest pose inside it
        assertEquals(700, h.at(50 * MS).x(), 1e-9);
        // Within the window it still interpolates
        h.record(800 * MS, new Pose(800, 0, 0));
        assertEquals(750, h.at(750 * MS).x(), 1e-9);
    }

    @Test
    public void clearForgetsEverything() {
        PoseHistory h = new PoseHistory();
        h.record(0, new Pose(1, 1, 0));
        h.clear();
        assertNull(h.at(0));
    }

    @Test
    public void oldPosesAreOverwrittenWhenFull() {
        PoseHistory h = new PoseHistory();
        for (int i = 0; i < 200; i++) h.record(i * 2 * MS, new Pose(i, 0, 0));
        assertEquals(150, h.at(150 * 2 * MS).x(), 1e-9);
        assertEquals(150.5, h.at(301 * MS).x(), 1e-9);
    }
}
