package org.firstinspires.ftc.teamcode.commands;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.helpers.MatchState;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class MatchStateTest {
    @Before
    @After
    public void clear() {
        MatchState.clear();
    }

    @Test
    public void neverSavedHasNoAge() {
        assertTrue(Double.isNaN(MatchState.ageSeconds()));
    }

    @Test
    public void savedHasASmallPositiveAge() throws InterruptedException {
        MatchState.save(Alliance.BLUE, new Pose(1, 2, 0));
        Thread.sleep(20);
        double age = MatchState.ageSeconds();
        assertTrue(age >= 0.015 && age < 5);
        assertEquals(Alliance.BLUE, MatchState.alliance());
    }
}
