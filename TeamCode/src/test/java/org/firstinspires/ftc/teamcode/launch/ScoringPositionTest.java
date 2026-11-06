package org.firstinspires.ftc.teamcode.launch;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.junit.Test;

public class ScoringPositionTest {
    @Test
    public void blueIsRedTurnedHalfWayRound() {
        double[][] points = {{30, 20}, {60, 50}, {90, 100}, {20, 130}, {72, 72}, {110, 15}};
        for (double[] p : points) {
            for (LaunchModel.Launcher l : LaunchModel.Launcher.values()) {
                assertEquals(ScoringPosition.chance(Alliance.RED, l, 144 - p[0], 144 - p[1]),
                        ScoringPosition.chance(Alliance.BLUE, l, p[0], p[1]), 1e-12);
            }
            assertEquals(ScoringPosition.chanceBoth(Alliance.RED, 144 - p[0], 144 - p[1]),
                    ScoringPosition.chanceBoth(Alliance.BLUE, p[0], p[1]), 1e-12);
        }
    }

    @Test
    public void chancesAreProbabilities() {
        for (double x = 0; x <= 144; x += 7) {
            for (double y = 0; y <= 144; y += 7) {
                double c = ScoringPosition.chanceBoth(Alliance.RED, x, y);
                assertTrue("(" + x + ", " + y + ") gave " + c, c >= 0 && c <= 1);
            }
        }
    }

    @Test
    public void unreachableCellsReadZero() {
        // A robot's centre can't be in the field's corner
        assertEquals(0.0, ScoringPosition.chanceBoth(Alliance.RED, 0, 0), 0);
        assertEquals(0.0, ScoringPosition.chanceBoth(Alliance.BLUE, 144, 144), 0);
    }

    @Test
    public void somewhereScoresLikely() {
        boolean found = false;
        for (double x = 0; x <= 144 && !found; x += 3) {
            for (double y = 0; y <= 144 && !found; y += 3) {
                found = ScoringPosition.grade(ScoringPosition.chanceBoth(Alliance.RED, x, y)) == ShotQuality.LIKELY;
            }
        }
        assertTrue(found);
    }

    @Test
    public void gradeUsesTheThresholds() {
        assertEquals(ShotQuality.LIKELY, ScoringPosition.grade(ScoringPosition.likelyChance + 0.01));
        assertEquals(ShotQuality.MARGINAL, ScoringPosition.grade(ScoringPosition.likelyChance - 0.01));
        assertEquals(ShotQuality.MARGINAL, ScoringPosition.grade(ScoringPosition.marginalChance + 0.01));
        assertEquals(ShotQuality.NONE, ScoringPosition.grade(ScoringPosition.marginalChance - 0.01));
    }

    @Test
    public void halves() {
        assertEquals(ScoringPosition.Half.AUDIENCE, ScoringPosition.halfOf(10));
        assertEquals(ScoringPosition.Half.SCORING, ScoringPosition.halfOf(100));
    }

    /**
     * Along each row, finds where the raw grade changes, then jitters the position back and forth across that
     * point. The raw grade flips every time, the graded result from update() must not.
     */
    @Test
    public void updateDoesNotFlickerAtAThreshold() {
        int crossings = 0;
        for (double y = 3; y <= 140; y += 1.5) {
            for (double x = 0.5; x < 143; x += 0.25) {
                double c0 = ScoringPosition.chanceBoth(Alliance.RED, x, y);
                double c1 = ScoringPosition.chanceBoth(Alliance.RED, x + 0.25, y);
                if (ScoringPosition.grade(c0) == ScoringPosition.grade(c1)) continue;
                // A step between grid points of at most the hysteresis, or the margin couldn't cover it
                if (Math.abs(c1 - c0) > ScoringPosition.hysteresis / 2) continue;
                ScoringPosition sp = new ScoringPosition();
                sp.update(Alliance.RED, x, y);
                sp.update(Alliance.RED, x, y);
                ShotQuality settled = sp.quality();
                for (int i = 0; i < 20; i++) {
                    sp.update(Alliance.RED, i % 2 == 0 ? x + 0.25 : x, y);
                    assertEquals("flickered at (" + x + ", " + y + ")", settled, sp.quality());
                }
                crossings++;
            }
        }
        assertTrue("found no threshold crossing to test", crossings > 0);
    }

    @Test
    public void updateStoresTheLastResult() {
        ScoringPosition sp = new ScoringPosition();
        ShotQuality q = sp.update(Alliance.RED, 60, 30);
        assertEquals(q, sp.quality());
        assertEquals(ScoringPosition.chanceBoth(Alliance.RED, 60, 30), sp.lastChance(), 1e-12);
        assertNotNull(q);
    }
}
