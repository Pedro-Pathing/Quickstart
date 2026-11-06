package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class HiveCellsTest {
    @Test
    public void cellFollowsFieldHalfLikeTrackCell() {
        assertEquals(HiveCells.Cell.BLUE_AUDIENCE, HiveCells.forRobot(Field.Alliance.BLUE, new Pose(100, 72, 0)));
        assertEquals(HiveCells.Cell.BLUE_FAR, HiveCells.forRobot(Field.Alliance.BLUE, new Pose(100, 71.9, 0)));
        assertEquals(HiveCells.Cell.RED_AUDIENCE, HiveCells.forRobot(Field.Alliance.RED, new Pose(20, 130, 0)));
        assertEquals(HiveCells.Cell.RED_FAR, HiveCells.forRobot(Field.Alliance.RED, new Pose(20, 10, 0)));
    }

    @Test
    public void tagsMapToTheirCell() {
        assertEquals(HiveCells.Cell.RED_FAR, HiveCells.forTag(30));
        assertEquals(HiveCells.Cell.BLUE_FAR, HiveCells.forTag(45));
        assertNull(HiveCells.forTag(46));
    }
}
