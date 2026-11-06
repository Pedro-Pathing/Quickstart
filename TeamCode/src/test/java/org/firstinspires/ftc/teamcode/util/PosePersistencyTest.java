package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Field;
import org.junit.Test;

public class PosePersistencyTest {
    @Test
    public void fallsBackUntilSavedAndClears() {
        PosePersistency.clear();
        Pose fallback = new Pose(1, 2, 3);
        assertFalse(PosePersistency.hasPose());
        assertSame(fallback, PosePersistency.getPose(fallback));
        assertEquals(Field.Alliance.BLUE, PosePersistency.getAlliance(Field.Alliance.BLUE));

        Pose saved = new Pose(10, 20, 0.5);
        PosePersistency.savePose(saved);
        PosePersistency.saveAlliance(Field.Alliance.RED);
        assertTrue(PosePersistency.hasPose());
        assertSame(saved, PosePersistency.getPose(fallback));
        assertEquals(Field.Alliance.RED, PosePersistency.getAlliance(Field.Alliance.BLUE));

        PosePersistency.clear();
        assertSame(fallback, PosePersistency.getPose(fallback));
    }
}
