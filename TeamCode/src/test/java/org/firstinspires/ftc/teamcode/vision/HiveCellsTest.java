package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class HiveCellsTest {
    @Test
    public void redPicksAudienceCellFromAudienceHalf() {
        double[] c = HiveCells.nearestCell(Alliance.RED, 30, 20);
        assertEquals(HiveCells.RED_HIVE_X, c[0], 1e-9);
        assertEquals(HiveCells.AUDIENCE_CELL_Y, c[1], 1e-9);
    }

    @Test
    public void redPicksScoringCellFromScoringHalf() {
        double[] c = HiveCells.nearestCell(Alliance.RED, 30, 120);
        assertEquals(HiveCells.RED_HIVE_X, c[0], 1e-9);
        assertEquals(HiveCells.SCORING_CELL_Y, c[1], 1e-9);
    }

    @Test
    public void bluePicksCellsOnBlueHive() {
        double[] audience = HiveCells.nearestCell(Alliance.BLUE, 100, 20);
        double[] scoring = HiveCells.nearestCell(Alliance.BLUE, 100, 120);
        assertEquals(HiveCells.BLUE_HIVE_X, audience[0], 1e-9);
        assertEquals(HiveCells.AUDIENCE_CELL_Y, audience[1], 1e-9);
        assertEquals(HiveCells.BLUE_HIVE_X, scoring[0], 1e-9);
        assertEquals(HiveCells.SCORING_CELL_Y, scoring[1], 1e-9);
    }

    @Test
    public void midLineGoesToTheAudienceCell() {
        assertEquals(HiveCells.AUDIENCE_CELL_Y, HiveCells.nearestCell(Alliance.RED, 10, 72)[1], 1e-9);
    }

    @Test
    public void headingPointsAtTheCell() {
        // Left of the red audience cell, level with it: heading 0 (+X)
        assertEquals(0.0, HiveCells.headingToNearestCell(Alliance.RED, 20, HiveCells.AUDIENCE_CELL_Y), 1e-9);
        // Directly below the red scoring cell (at y 89.8), so it looks +Y: heading +90 degrees
        assertEquals(Math.PI / 2, HiveCells.headingToNearestCell(Alliance.RED, HiveCells.RED_HIVE_X, 80), 1e-9);
        // Right of the blue scoring cell, level with it: heading 180 degrees
        assertEquals(Math.PI,
                Math.abs(HiveCells.headingToNearestCell(Alliance.BLUE, 130, HiveCells.SCORING_CELL_Y)), 1e-9);
    }

    @Test
    public void headingMatchesAtan2OfTheCellOffset() {
        double[] c = HiveCells.nearestCell(Alliance.RED, 40, 90);
        assertEquals(Math.atan2(c[1] - 90, c[0] - 40), HiveCells.headingToNearestCell(Alliance.RED, 40, 90), 1e-12);
    }

    @Test
    public void blueIsRedTurnedHalfWayRound() {
        double[] red = HiveCells.nearestCell(Alliance.RED, 30, 20);
        double[] blue = HiveCells.nearestCell(Alliance.BLUE, 144 - 30, 144 - 20);
        assertEquals(144 - red[0], blue[0], 1e-9);
        assertEquals(144 - red[1], blue[1], 1e-9);
    }
}
